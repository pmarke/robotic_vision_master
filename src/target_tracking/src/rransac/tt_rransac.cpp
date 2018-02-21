#include "rransac/tt_rransac.h"




namespace robotic_vision {

TTRRansac::TTRRansac() {


    // Set parameters


      // populate plotting colors
    colors_ = std::vector<cv::Scalar>();
    for (int i = 0; i < 1000; i++)
        colors_.push_back(cv::Scalar(std::rand() % 256, std::rand() % 256, std::rand() % 256));

    text_scale_ = 2;

    // Used for display
    cv::namedWindow("Tracks", CV_WINDOW_NORMAL);
    cv::resizeWindow("Tracks", 1680/3, 1050/3);
    cv::moveWindow("Tracks",   2*1680/3, 1050/3);


    set_parameters();




}


void TTRRansac::add_measurments(std::vector<cv::Point2f>& pos, std::vector<cv::Point2f> vel, int source ) {


    // std::cout << "Add measurements" << std::endl;

    // for (int i = 0; i < pos.size(); i++ ) {
    //  std::cout << "pos x" << pos[i].x << std::endl;
    //  std::cout << "pos y" << pos[i].y << std::endl;
    //  std::cout << "vel x" << vel[i].x << std::endl;
    //  std::cout << "vel y" << vel[i].y << std::endl;
    // }

    tracker_.add_measurements<OpenCVPointAccess>(pos, vel, source);

    // std::cout << "measurements added" << std::endl;



}

void TTRRansac::add_measurments(std::vector<cv::Point2f>& pos, int source ) {

    
    tracker_.add_measurements<OpenCVPointAccess>(pos, source);
    

}

void TTRRansac::run_tracker() {

    // std::cout << "run tracker" << std::endl;

    models_ = tracker_.run();

    // std::cout << "models: " << models_.size() << std::endl;

    // clear history
    tracks_.clear();

    // temp track
    Tracks track;

    // unpack
    for (int i=0; i <models_.size(); i++) {

     track.id = models_[i]->GMN;
     track.inlier_ratio=models_[i]->rho;

     // position
     track.x = models_[i]->xhat(0);
     track.y = models_[i]->xhat(1);

     // velocity
     track.vx = models_[i]->xhat(2);
     track.vy = models_[i]->xhat(3);

     tracks_.push_back(track);



    }

    // std::cout << "tracker ran" << std::endl;

}

// void TTRRansac::apply_tranformation(Eigen::Projective2d& T) {
//   tracker_.apply_transformation(T);
// }

void TTRRansac::draw_tracks(const cv::Mat img) {

    cv::Mat altered_img;

    altered_img = img.clone();

    // std::cout << "Models: " << models_.size() << std::endl;

    for (int i = 0; i < models_.size(); i++) {

        cv::Scalar color = colors_[models_[i]->GMN];


        // center
        cv::Point center;
        center.x = models_[i]->xhat(0);
        center.y = models_[i]->xhat(1);


        // radius
        double radius = 50;//params_.tauR;

        // Draw circle around target
        cv::circle(altered_img, center, (int) radius, color, 2,8,0);
        cv::circle(altered_img, center, 2,cv::Scalar(0, 0, 255), 2, 8, 0 );

        // draw velocity vector
        cv::Point velocity;
        double velocity_scale = 10; // for visibility
        velocity.x = models_[i]->xhat(2) * velocity_scale;
        velocity.y = models_[i]->xhat(3) * velocity_scale;
        cv::line(altered_img, center, center + velocity, color, 1, CV_AA);






        // draw model number and inlier ratio
        std::stringstream ssGMN;
        ssGMN << models_[i]->GMN;
        int boldness = 2*(int)text_scale_;
        cv::putText(altered_img, ssGMN.str().c_str(), cv::Point(center.x + 5, center.y + 15), cv::FONT_HERSHEY_SIMPLEX, 0.85*text_scale_, cv::Scalar(0, 0, 210), boldness);

    


    }

    cv::imshow("Tracks", altered_img);
    cv::waitKey(1);



}

void TTRRansac::set_parameters() {

    // // general
    // params_.dt = 5/30.0;
    params_.dt = 1;

    // motion model specific parameters
    params_.sigmaQ_vel = 3;
    params_.alphaQ_vel = 0.5;
    params_.sigmaQ_jrk = 0.0075;
    params_.alphaQ_jrk = 0.5;

    // R-RANSAC specific parameters
    params_.Nw = 10;
    params_.M = 30;
    params_.tauR = 50.0;
    params_.set_motion_model(rransac::core::MotionModelType::CONSTANT_JERK);

    // // RANSAC specific parameters
    params_.ell = 100;
    params_.guided_sampling_threshold = 1000;
    params_.tauR_RANSAC = 50;
    params_.gamma = 0.5;
    params_.set_motion_model_RANSAC(rransac::core::MotionModelType::CONSTANT_VEL);

    // // model merging parameters
    params_.tau_vel_percent_diff = 0.25;
    params_.tau_vel_abs_diff = 900;
    params_.tau_angle_abs_diff = 15;
    params_.tau_xpos_abs_diff = 50;
    params_.tau_ypos_abs_diff = 50;

    // // model pruning parameters
    
    params_.field_max_x = std::abs(1280);
    params_.field_max_y = std::abs(720);
    params_.tau_CMD_prune = 10;

    // // track (i.e., Good Model) parameters
    params_.tau_rho = 0.6;
    params_.tau_CMD = 4;
    params_.tau_Vmax = 30;
    params_.tau_T = 4;


    params_.add_source(0, true, 20, 20);
    params_.add_source(1, false, 20, 20);
    params_.add_source(2, true, 20, 20);

  // Update the R-RANSAC Tracker with these new parameters
  tracker_.set_parameters(params_);

}





}