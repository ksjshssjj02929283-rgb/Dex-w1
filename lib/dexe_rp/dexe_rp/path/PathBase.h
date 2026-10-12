/*
 * ----------------------------------------------------------------------------
 * Copyright (c) 2021-2026 DexForce Technology Co., Ltd.
 *
 * All rights reserved.
 * ----------------------------------------------------------------------------
 */

#pragma once

#include "dexe_rp/common/CommonHeader.h"
#include "dexe_rp/path/Types.h"

namespace DexeRP {
namespace Path {

template <typename _LieGroup>
class PathBase {
private:
    using Scalar = typename _LieGroup::Scalar;
    using Tangent = typename _LieGroup::Tangent;

public:
    virtual ~PathBase() = default;
    // non-copyable to enforce explicit cloning semantics
    PathBase(const PathBase&) = delete;
    PathBase& operator=(const PathBase&) = delete;
    // movable
    PathBase(PathBase&&) noexcept = default;
    PathBase& operator=(PathBase&&) noexcept = default;
    // deep copy
    virtual std::shared_ptr<PathBase<_LieGroup>> Clone() const = 0;

    virtual _LieGroup GetConfig(Scalar s) const = 0;
    virtual Tangent GetTangent(Scalar s) const = 0;
    virtual Tangent GetCurvature(Scalar s) const = 0;
    virtual Tangent GetTorsion(Scalar s) const = 0;

    Scalar GetLength() const { return length_; }

    PathPoint<_LieGroup> Eval(Scalar s) const {
        return PathPoint<_LieGroup>{GetConfig(s), GetTangent(s), GetCurvature(s), GetTorsion(s)};
    }

    std::vector<PathPoint<_LieGroup>> SampleByStep(Scalar delta_s) const {
        std::vector<PathPoint<_LieGroup>> samples;
        auto s_samples = DexeRP::SampleByStep(0.0, this->length_, delta_s);
        samples.reserve(s_samples.size());
        for (const auto& s : s_samples) {
            samples.emplace_back(Eval(s));
        }
        return samples;
    }

    std::vector<PathPoint<_LieGroup>> SampleByNum(int num_samples) const {
        std::vector<PathPoint<_LieGroup>> samples;
        auto s_samples = DexeRP::SampleByNum(0.0, this->length_, num_samples);
        samples.reserve(s_samples.size());
        for (const auto& s : s_samples) {
            samples.emplace_back(Eval(s));
        }
        return samples;
    }

protected:
    explicit PathBase(Scalar length) : length_(length) {
        DEXERP_ENSURE(length >= 0.0, "Path length must be non-negative!");
    }

    Scalar length_{0.0};
};

template <typename _LieGroup>
class CompoundPath : public PathBase<_LieGroup> {
private:
    using Scalar = typename _LieGroup::Scalar;
    using Tangent = typename _LieGroup::Tangent;

public:
    using SegmentPtr = std::shared_ptr<PathBase<_LieGroup>>;
    using SegmentConstPtr = std::shared_ptr<const PathBase<_LieGroup>>;

    CompoundPath(const std::vector<SegmentPtr>& segments) : PathBase<_LieGroup>(0.0) {
        DEXERP_ENSURE(!segments.empty(), "Compound path must have at least one segment!");
        length_prefix_.emplace_back(0.0);
        AddPathSegments(segments);
    }

    virtual ~CompoundPath() = default;

    virtual SegmentPtr Clone() const override {
        std::vector<SegmentPtr> segs;
        segs.reserve(segments_.size());
        for (const auto& seg : segments_) {
            segs.push_back(seg->Clone());
        }
        return std::make_shared<CompoundPath<_LieGroup>>(segs);
    }

    virtual _LieGroup GetConfig(Scalar s) const override {
        auto [segment, local_s] = _GetSegmentAtS(s);
        return segment->GetConfig(local_s);
    }

    virtual Tangent GetTangent(Scalar s) const override {
        auto [segment, local_s] = _GetSegmentAtS(s);
        return segment->GetTangent(local_s);
    }

    virtual Tangent GetCurvature(Scalar s) const override {
        auto [segment, local_s] = _GetSegmentAtS(s);
        return segment->GetCurvature(local_s);
    }

    virtual Tangent GetTorsion(Scalar s) const override {
        auto [segment, local_s] = _GetSegmentAtS(s);
        return segment->GetTorsion(local_s);
    }

    const std::vector<SegmentPtr>& GetPathSegments() const { return segments_; }

    void AddPathSegment(const SegmentConstPtr& segment) { AddPathSegments(std::vector<SegmentConstPtr>{segment}); }

    void AddPathSegment(const SegmentPtr& segment) { AddPathSegments(std::vector<SegmentPtr>{segment}); }

    void AddPathSegments(const std::vector<SegmentConstPtr>& segments) {
        std::vector<SegmentPtr> segments_cloned;
        segments_cloned.reserve(segments.size());
        for (const auto& seg : segments) {
            segments_cloned.push_back(seg->Clone());
        }
        return AddPathSegments(segments_cloned);
    }

    void AddPathSegments(const std::vector<SegmentPtr>& segments) {
        segments_.reserve(segments_.size() + segments.size());
        length_prefix_.reserve(segments_.size() + segments.size() + 1);
        Scalar total_length = this->length_;
        for (auto& seg : segments) {
            total_length += seg->GetLength();
            segments_.emplace_back(std::move(seg));
            length_prefix_.emplace_back(total_length);
        }
        this->length_ = total_length;
    }

protected:
    std::pair<SegmentPtr, Scalar> _GetSegmentAtS(Scalar s) const {
        if (this->length_ <= kEpsilon) return {segments_.front(), 0.0};
        const Scalar s_ = std::clamp(s, 0.0, this->length_);
        auto it = std::upper_bound(length_prefix_.begin(), length_prefix_.end(), s_);
        size_t idx = std::distance(length_prefix_.begin(), it);
        idx = std::clamp<size_t>(idx, 1, segments_.size()) - 1;
        return {segments_[idx], s_ - length_prefix_[idx]};
    }

    std::vector<SegmentPtr> segments_;
    std::vector<Scalar> length_prefix_;
};

}  // namespace Path
}  // namespace DexeRP
