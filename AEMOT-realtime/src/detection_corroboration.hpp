#pragma once
#include <cmath>
#include <vector>

// Requires two spatially-close, temporally-close positive detections before
// signaling "spawn a track" - filters isolated single-shot noise without
// weakening the regression test itself. Cell size should be roughly the
// expected object footprint

class DetectionCorroborationGrid {

public:
    DetectionCorroborationGrid(int width, int height, double cell_size_px,
                                double corroboration_window, 
                                double direction_cos_threshold)
        : cell_size_(cell_size_px),
            corroboration_window_(corroboration_window),
            direction_cos_threshold_(direction_cos_threshold)
    {
        cell_cols_ = static_cast<int>(std::ceil(width / cell_size_px)) + 1;
        cell_rows_ = static_cast<int>(std::ceil(height / cell_size_px)) + 1;
        cell_ts_.assign(cell_cols_ * cell_rows_, -1e18);
        cell_lx_.assign(cell_cols_ * cell_rows_, 0.0);
        cell_ly_.assign(cell_cols_ * cell_rows_, 0.0);
    }

    // returns true if this corroborates a recent neaby one. always records this detection's cell
    // regardless, so future checks can reference it
    bool check_and_update(double x, double y, double ts, double lx, double ly) {
        const int cx = static_cast<int>(x / cell_size_);
        const int cy = static_cast<int>(y / cell_size_);
        bool corroborated = false;

        for (int dy = -1; dy <= 1 && !corroborated; dy++) {
            const int ny = cy + dy;
            if (ny < 0 || ny >= cell_rows_) continue;
            for (int dx = -1; dx <= 1; dx++) {
                const int nx = cx + dx;
                if (nx < 0 || nx >= cell_cols_) continue;
                const int idx = ny * cell_cols_ + nx;
                const double age = ts - cell_ts_[idx];
                if (age < 0.0 || age > corroboration_window_) continue;

                const double cos_sim = lx * cell_lx_[idx] + ly * cell_ly_[idx];
                if (cos_sim >= direction_cos_threshold_) {
                    corroborated = true;
                    break;
                }
            }
        }

        const int idx = cy * cell_cols_ + cx;
        cell_ts_[idx] = ts;
        cell_lx_[idx] = lx;
        cell_ly_[idx] = ly;
        return corroborated;
    }

private:
    int cell_cols_, cell_rows_;
    double cell_size_;
    double corroboration_window_;
    double direction_cos_threshold_;
    std::vector<double> cell_ts_, cell_lx_, cell_ly_;
};
