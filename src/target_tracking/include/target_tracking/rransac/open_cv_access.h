    #pragma once

    class OpenCVPointAccess : public rransac::BaseAccessType<cv::Point2f>
    {
    public:
        OpenCVPointAccess() {};
        ~OpenCVPointAccess() {};

    private:
        const double x(const cv::Point2f& p) { return (double)p.x; }
        const double y(const cv::Point2f& p) { return (double)p.y; }
    };
