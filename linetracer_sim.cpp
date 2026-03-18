#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <vector>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include "dxl3/dxl.hpp"    // ← dxl3 패키지 헤더

// ========= 비블로킹 키 입력 함수 =========
int kbhit()
{
    struct termios oldt, newt;
    int ch, oldf;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);
    if (ch != EOF) {
        ungetc(ch, stdin);
        return 1;
    }
    return 0;
}

int getch()
{
    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

class linetracer_sim : public rclcpp::Node
{
public:
    linetracer_sim()
    : Node("linetracer_sim"),
      prev_center_x_(-1.0),
      lost_count_(0),
      mode_(false),
      base_speed_(100),
      k_(1.0),
      log_count_(0),
      frame_count_(0),
      vel1_(0), vel2_(0),
      goal1_(0), goal2_(0)
    {
        // ========= Dynamixel 초기화 =========
        if (!mx_.open()) {
            RCLCPP_ERROR(this->get_logger(), "dxl open error");
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Dynamixel opened successfully");

        // ========= 이미지 구독 =========
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "video1", rclcpp::QoS(10),
            std::bind(&linetracer_sim::image_callback, this,
                      std::placeholders::_1));

        // ========= 속도명령 퍼블리셔 =========
        vel_publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(
            "vel_cmd", rclcpp::QoS(10));

        // ========= 50ms 타이머 (가감속 처리) =========
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&linetracer_sim::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "linetracer_sim Node Started");
        RCLCPP_INFO(this->get_logger(), "'s' : start  |  'q' : stop");
    }

    ~linetracer_sim()
    {
        // 종료 시 모터 정지
        mx_.setVelocity(0, 0);
        mx_.close();
        RCLCPP_INFO(this->get_logger(), "Dynamixel closed");
    }

private:
    // ========= 50ms 타이머 → 가감속 + 모터 제어 =========
    void timer_callback()
    {
        // 가감속 처리 (actuator main.cpp 동일)
        if      (goal1_ > vel1_) vel1_ += 5;
        else if (goal1_ < vel1_) vel1_ -= 5;
        else                     vel1_  = goal1_;

        if      (goal2_ > vel2_) vel2_ += 5;
        else if (goal2_ < vel2_) vel2_ -= 5;
        else                     vel2_  = goal2_;

        // 모터 속도 명령 전송
        if (!mx_.setVelocity(vel1_, vel2_)) {
            RCLCPP_ERROR(this->get_logger(), "setVelocity error");
        }
    }

    // ========= 이미지 콜백 =========
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        auto start = std::chrono::steady_clock::now();
        cv::Mat frame;

        try {
            frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (...) { return; }
        if (frame.empty()) return;

        int frame_w = frame.cols;
        int frame_h = frame.rows;

        // ========= ROI 영역 설정 =========
        int roi_width  = 640;
        int roi_height = 90;
        int roi_x      = 0;
        int roi_y      = frame_h - roi_height;
        cv::Rect roi_rect(roi_x, roi_y, roi_width, roi_height);

        // ========= 밝기 보정 =========
        cv::Mat roi = frame(roi_rect);
        cv::Mat roi_adjusted;
        const double target_mean = 140.0;
        double shift = target_mean - cv::mean(roi)[0];
        roi.convertTo(roi_adjusted, -1, 0.7, shift);

        // ========= 흑백 + 이진화 =========
        cv::Mat gray, binary;
        cv::cvtColor(roi_adjusted, gray, cv::COLOR_BGR2GRAY);
        cv::threshold(gray, binary, 140, 255, cv::THRESH_BINARY);

        // ========= 디스플레이용 이미지 =========
        cv::Mat display;
        cv::cvtColor(binary, display, cv::COLOR_GRAY2BGR);

        // ========= 컨투어 검출 =========
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binary, contours, cv::RETR_EXTERNAL,
                         cv::CHAIN_APPROX_SIMPLE);

        if (prev_center_x_ < 0)
            prev_center_x_ = frame_w / 2.0;

        double best_center_x = prev_center_x_;
        double min_dist      = 1e9;
        cv::Rect best_rect;
        bool found = false;

        // ========= 각 컨투어 분석 =========
        for (const auto &cnt : contours)
        {
            cv::Rect rect = cv::boundingRect(cnt);
            double area   = rect.area();
            int cx        = rect.x + rect.width / 2;

            cv::rectangle(display, rect, cv::Scalar(255, 0, 0), 2);

            if (area < 60) continue;
            if (rect.width < 3 || rect.height < 5) continue;
            if ((float)rect.height / rect.width < 0.10f) continue;
            if (rect.y + rect.height < roi_height * 0.30) continue;

            double candidate_x = roi_rect.x + cx;
            double dist        = std::abs(candidate_x - prev_center_x_);

            if (dist < min_dist) {
                min_dist      = dist;
                best_center_x = candidate_x;
                best_rect     = rect;
                found         = true;
            }
        }

        // ========= 추적 상태 파라미터 =========
        const double max_jump       = 160.0;
        const int    max_lost       = 5;
        const double max_speed      = 12.0;
        const double reappear_speed = 6.0;

        // ========= 라인을 찾았을 때 =========
        if (found && min_dist < max_jump)
        {
            lost_count_ = 0;

            cv::rectangle(display, best_rect, cv::Scalar(0, 0, 255), 2);
            cv::circle(display,
                cv::Point(best_center_x - roi_rect.x,
                          best_rect.y + best_rect.height / 2),
                6, cv::Scalar(0, 0, 255), -1);

            double target = best_center_x;
            double dx     = target - prev_center_x_;
            double limit  = (lost_count_ > 0) ? reappear_speed : max_speed;

            if (std::abs(dx) > limit)
                target = prev_center_x_ + (dx > 0 ? limit : -limit);

            double alpha   = 0.3;
            prev_center_x_ = prev_center_x_ * (1.0 - alpha) + target * alpha;
        }
        else
        {
            // ========= 라인을 못 찾을 때 =========
            lost_count_++;
            if (lost_count_ > max_lost)
                lost_count_ = max_lost;

            double target_x;
            if (prev_center_x_ < frame_w * 0.3)
                target_x = roi_rect.x + (roi_rect.width * 3.0 / 5.0);
            else if (prev_center_x_ > frame_w * 0.7)
                target_x = roi_rect.x + (roi_rect.width * 1.0 / 5.0);
            else
                target_x = prev_center_x_;

            prev_center_x_ = prev_center_x_ * 0.85 + target_x * 0.15;

            cv::circle(display,
                cv::Point(prev_center_x_ - roi_rect.x, roi_height - 10),
                6, cv::Scalar(0, 0, 255), -1);
        }

        // ========= error 계산 =========
        double error = frame_w / 2.0 - prev_center_x_;

        // ========= P제어 속도 계산 =========
        // 직진 : lvel=+100, rvel=-100
        // error > 0 (라인 왼쪽) → 우회전
        // error < 0 (라인 오른쪽) → 좌회전
        int lvel = static_cast<int>( base_speed_ - k_ * error);
        int rvel = static_cast<int>(-(base_speed_ - k_ * error));

        // ========= s/q 키 처리 =========
        if (kbhit())
        {
            int ch = getch();
            if (ch == 'q') {
                mode_ = false;
                goal1_ = 0;
                goal2_ = 0;
                RCLCPP_INFO(this->get_logger(), "STOP");
            } else if (ch == 's') {
                mode_ = true;
                RCLCPP_INFO(this->get_logger(), "START");
            }
        }

        // ========= goal 속도 업데이트 (타이머에서 가감속 처리) =========
        if (mode_) {
            goal1_ = lvel;
            goal2_ = rvel;
        } else {
            goal1_ = 0;
            goal2_ = 0;
        }

        // ========= vel_cmd 토픽 퍼블리시 =========
        auto vel_msg = std_msgs::msg::Int32MultiArray();
        vel_msg.data.resize(2);
        vel_msg.data[0] = vel1_;   // 현재 실제 속도
        vel_msg.data[1] = vel2_;
        vel_publisher_->publish(vel_msg);

        // ========= 수행 시간 측정 =========
        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = end - start;

        // ========= 10프레임마다 터미널 출력 =========
        log_count_++;
        if (log_count_ % 10 == 0)
        {
            RCLCPP_INFO(this->get_logger(),
                "err:%d, lvel:%d, rvel:%d, time:%.4f sec",
                (int)error, vel1_, vel2_, elapsed.count());
        }

        // ========= 3프레임마다 화면 갱신 =========
        frame_count_++;
        if (frame_count_ % 3 == 0)
        {
            cv::imshow("Original Frame", frame);
            cv::imshow("Binary with Overlay", display);
            cv::waitKey(1);
        }
    }

    // 라인 검출 멤버
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr vel_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    double prev_center_x_;
    int    lost_count_;
    bool   mode_;
    int    base_speed_;
    double k_;
    int    log_count_;
    int    frame_count_;

    // Dynamixel 멤버
    Dxl mx_;
    int vel1_, vel2_;     // 현재 실제 속도 (가감속 적용)
    int goal1_, goal2_;   // 목표 속도
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<linetracer_sim>());
    rclcpp::shutdown();
    return 0;
}
