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

#include <Eigen/Core>

template <typename T, int Row, int Col, int Options, int MaxRow, int MaxCol>
struct fmt::is_range<Eigen::Matrix<T, Row, Col, Options, MaxRow, MaxCol>, char> : std::false_type {};

template <typename T, int Row, int Col, int Options, int MaxRow, int MaxCol>
struct fmt::formatter<Eigen::Matrix<T, Row, Col, Options, MaxRow, MaxCol>> {
    constexpr auto parse(fmt::format_parse_context& ctx) {
        auto it = ctx.begin();
        auto end = ctx.end();
        while (it != end && *it != '}') ++it;  // ignore all format specs
        return it;
    }

    template <typename FormatContext>
    auto format(const Eigen::Matrix<T, Row, Col, Options, MaxRow, MaxCol>& m, FormatContext& ctx) const {
        auto out = ctx.out();

        const int rows = static_cast<int>(m.rows());
        const int cols = static_cast<int>(m.cols());

        // CVec
        if (rows == 1 || cols == 1) {
            const int n = rows * cols;
            fmt::format_to(out, "[");
            for (int i = 0; i < n; ++i) {
                fmt::format_to(out, "{}", m(i));
                if (i + 1 < n) fmt::format_to(out, ", ");
            }
            fmt::format_to(out, "]");
            return out;
        }

        // Mat
        fmt::format_to(out, "[");
        for (int r = 0; r < rows; ++r) {
            fmt::format_to(out, "[");
            for (int c = 0; c < cols; ++c) {
                fmt::format_to(out, "{}", m(r, c));
                if (c + 1 < cols) fmt::format_to(out, ", ");
            }
            fmt::format_to(out, "]");
            if (r + 1 < rows) fmt::format_to(out, ", ");
        }
        fmt::format_to(out, "]");

        return out;
    }
};
