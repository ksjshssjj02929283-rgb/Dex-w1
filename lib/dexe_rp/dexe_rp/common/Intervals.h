/*
 * ----------------------------------------------------------------------------
 * Copyright (c) 2021-2026 DexForce Technology Co., Ltd.
 *
 * All rights reserved.
 * ----------------------------------------------------------------------------
 */

#pragma once

#include <fmt/format.h>
#include <fmt/ranges.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <initializer_list>
#include <utility>
#include <vector>

#include "dexe_rp/common/Utils.h"

namespace DexeRP {

/**
 * @brief A single closed 1-D interval [lb, ub].
 *
 *   - The interval is "empty" iff lb > ub.
 *   - The default-constructed interval is empty (lb = 1, ub = 0).
 *   - All functions return false (or an empty result) when interval is empty.
 *   - Endpoint comparisons use kEpsSmall as absolute tolerance.
 */
struct Interval {
    double lb;  ///< lower bound (inclusive)
    double ub;  ///< upper bound (inclusive)

    Interval() : lb(1.0), ub(0.0) {}
    Interval(double lb, double ub) : lb(lb), ub(ub) {}
    Interval(const Interval&) = default;
    Interval& operator=(const Interval&) = default;
    ~Interval() = default;

    /// Factory: an explicitly empty interval.
    static Interval Empty() { return Interval(); }

    // ---- queries ----

    bool IsEmpty() const { return lb > ub; }

    double Length() const { return IsEmpty() ? 0.0 : ub - lb; }

    /**
     * @note Check !IsEmpty() first; Otherwise throw error
     */
    double Midpoint() const {
        if (IsEmpty()) {
            DEXERP_THROW("Cannot compute midpoint of an empty interval.");
        }
        return 0.5 * (lb + ub);
    }

    bool IsContain(double x) const { return !IsEmpty() && x >= lb - kEpsSmall && x <= ub + kEpsSmall; }

    bool IsContain(const Interval& other) const {
        if (IsEmpty() || other.IsEmpty()) return false;
        return lb <= other.lb + kEpsSmall && ub >= other.ub - kEpsSmall;
    }

    bool IsOverlap(const Interval& other) const {
        if (IsEmpty() || other.IsEmpty()) return false;
        return !(ub + kEpsSmall < other.lb || other.ub + kEpsSmall < lb);
    }

    // ---- combinators ----

    Interval Intersect(const Interval& other) const {
        if (IsEmpty() || other.IsEmpty()) return Interval::Empty();
        return Interval(std::max(lb, other.lb), std::min(ub, other.ub));
    }

    // ---- sampling ----

    std::vector<double> SampleByStep(double step) const { return DexeRP::SampleByStep(lb, ub, step); }

    std::vector<double> SampleByNum(size_t n) const { return DexeRP::SampleByNum(lb, ub, n); }

    // ---- comparison ----

    bool operator==(const Interval& other) const { return lb == other.lb && ub == other.ub; }
    bool operator!=(const Interval& other) const { return !(*this == other); }
};

/**
 * @brief An ordered set of real numbers represented as disjoint closed intervals.
 *
 * Canonical-form invariant (always maintained internally):
 *   - contains no empty intervals,
 *   - intervals are sorted by lb ascending,
 *   - intervals are pairwise disjoint and non-touching
 *     (overlapping or touching inputs are merged on construction).
 */
class Intervals {
public:
    using value_type = Interval;
    using const_iterator = std::vector<Interval>::const_iterator;

    Intervals() = default;
    ~Intervals() = default;

    Intervals(const Intervals&) = default;
    Intervals& operator=(const Intervals&) = default;

    Intervals(Intervals&&) = default;
    Intervals& operator=(Intervals&&) = default;

    /// From a single interval (empty interval -> empty set).
    Intervals(const Interval& interval) {
        if (!interval.IsEmpty()) {
            data_.push_back(interval);
        }
    }

    /// From a brace list, e.g. {Interval(0,1), Interval(2,3)}
    Intervals(std::initializer_list<Interval> intervals) : data_(intervals) { Normalize(); }

    /// From an arbitrary list
    explicit Intervals(std::vector<Interval> intervals) : data_(std::move(intervals)) { Normalize(); }

    /**
     * @brief Build a set from candidate split points by checking each segment.
     *
     * The split points are sorted and de-duplicated; for every consecutive pair
     * (p_i, p_{i+1}) the segment is kept if func(midpoint) is true. The kept
     * segments are returned as a normalized (merged) set.
     *
     * @param split_points boundary candidates
     * @param func         check function at each segment midpoint.
     */
    template <typename Func>
    static Intervals FromSplitPoints(std::vector<double> split_points, Func&& f) {
        std::sort(split_points.begin(), split_points.end());

        // de-duplicate within tolerance
        std::vector<double> sp;
        sp.reserve(split_points.size());
        for (double x : split_points) {
            if (sp.empty() || std::abs(x - sp.back()) > kEpsSmall) {
                sp.push_back(x);
            }
        }
        if (sp.size() < 2) return {};

        // check
        std::vector<Interval> segs;
        segs.reserve(sp.size() - 1);
        for (size_t i = 1; i < sp.size(); ++i) {
            const double mid = 0.5 * (sp[i - 1] + sp[i]);
            if (f(mid)) {
                segs.emplace_back(sp[i - 1], sp[i]);
            }
        }
        return Intervals(std::move(segs));
    }

    static Intervals IntersectAll(const std::vector<Intervals>& list) {
        if (list.empty()) return {};
        Intervals cur = list[0];
        for (size_t k = 1; k < list.size(); ++k) {
            cur = cur.Intersect(list[k]);
            if (cur.IsEmpty()) break;
        }
        return cur;
    }

    // ---- read-only container access (canonical order) ----

    bool IsEmpty() const { return data_.empty(); }
    size_t Size() const { return data_.size(); }
    const Interval& operator[](size_t i) const { return data_[i]; }
    const Interval& Front() const { return data_.front(); }
    const Interval& Back() const { return data_.back(); }
    const_iterator begin() const { return data_.begin(); }
    const_iterator end() const { return data_.end(); }
    const std::vector<Interval>& Data() const { return data_; }
    void Clear() { data_.clear(); }

    // ---- set algebra (const, return a normalized result) ----

    Intervals Union(const Intervals& other) const {
        std::vector<Interval> v = data_;
        v.insert(v.end(), other.data_.begin(), other.data_.end());
        return Intervals(std::move(v));
    }

    Intervals Intersect(const Intervals& other) const {
        Intervals result;
        const std::vector<Interval>& a = data_;
        const std::vector<Interval>& b = other.data_;
        size_t i = 0, j = 0;
        while (i < a.size() && j < b.size()) {
            const Interval t = a[i].Intersect(b[j]);
            if (!t.IsEmpty() && t.ub - t.lb > kEpsSmall) result.data_.push_back(t);

            if (a[i].ub < b[j].ub - kEpsSmall)
                ++i;
            else if (b[j].ub < a[i].ub - kEpsSmall)
                ++j;
            else {
                ++i;
                ++j;
            }
        }
        return result;
    }

    Intervals Difference(const Intervals& other) const {
        Intervals result;
        for (const Interval& a : data_) {
            const Intervals piece = other.Complement(a);
            result.data_.insert(result.data_.end(), piece.data_.begin(), piece.data_.end());
        }
        return result;
    }

    Intervals Complement(const Interval& domain) const {
        Intervals result;
        if (domain.IsEmpty()) return result;

        double cur = domain.lb;
        for (const Interval& itv : data_) {
            if (itv.ub < domain.lb) continue;
            if (itv.lb > domain.ub) break;

            const double l = std::max(cur, domain.lb);
            const double r = std::min(itv.lb, domain.ub);
            if (l <= r - kEpsSmall) result.data_.emplace_back(l, r);

            cur = std::max(cur, itv.ub);
        }
        if (cur < domain.ub - kEpsSmall) result.data_.emplace_back(cur, domain.ub);
        return result;
    }

    Intervals& operator|=(const Intervals& other) {
        *this = Union(other);
        return *this;
    }
    Intervals& operator&=(const Intervals& other) {
        *this = Intersect(other);
        return *this;
    }
    Intervals& operator-=(const Intervals& other) {
        *this = Difference(other);
        return *this;
    }

    void Add(const Interval& interval) {
        if (interval.IsEmpty()) return;
        data_.push_back(interval);
        Normalize();
    }

    // ---- queries ----

    double Length() const {
        double sum = 0.0;
        for (const Interval& it : data_) sum += it.Length();
        return sum;
    }

    bool IsContain(double x) const { return Find(x) != nullptr; }

    const Interval* Find(double x) const {
        for (const Interval& it : data_) {
            if (it.lb - kEpsSmall > x) break;
            if (it.IsContain(x)) return &it;
        }
        return nullptr;
    }

private:
    /// Restore the canonical-form invariant on data_.
    void Normalize() {
        std::vector<Interval> v;
        v.reserve(data_.size());
        for (const Interval& it : data_) {
            if (!it.IsEmpty()) v.push_back(it);
        }
        if (v.empty()) {
            data_.clear();
            return;
        }

        std::sort(v.begin(), v.end(), [](const Interval& a, const Interval& b) {
            if (a.lb != b.lb) return a.lb < b.lb;
            return a.ub < b.ub;
        });

        std::vector<Interval> merged;
        merged.reserve(v.size());
        merged.push_back(v[0]);
        for (size_t k = 1; k < v.size(); ++k) {
            Interval& pre = merged.back();
            if (v[k].lb <= pre.ub + kEpsSmall)  // overlap or touch
                pre.ub = std::max(pre.ub, v[k].ub);
            else
                merged.push_back(v[k]);
        }
        data_ = std::move(merged);
    }

    std::vector<Interval> data_;  ///< canonical: non-empty, sorted, disjoint
};

// ---- free binary operators (mirror Union / Intersect / Difference) ----

inline Intervals operator|(const Intervals& a, const Intervals& b) { return a.Union(b); }
inline Intervals operator&(const Intervals& a, const Intervals& b) { return a.Intersect(b); }
inline Intervals operator-(const Intervals& a, const Intervals& b) { return a.Difference(b); }

}  // namespace DexeRP

template <>
struct fmt::formatter<DexeRP::Interval> {
    constexpr auto parse(format_parse_context& ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const DexeRP::Interval& itv, FormatContext& ctx) const {
        if (itv.IsEmpty()) return fmt::format_to(ctx.out(), "∅");
        return fmt::format_to(ctx.out(), "[{}, {}]", itv.lb, itv.ub);
    }
};
