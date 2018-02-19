    #pragma once

    class OpenCVPointAccess : public rransac::BaseAccessType<cv::Point2d>
    {
    public:
        OpenCVPointAccess() {};
        ~OpenCVPointAccess() {};

    private:
        const double x(const cv::Point2d& p) { return p.x; }
        const double y(const cv::Point2d& p) { return p.y; }
    };
