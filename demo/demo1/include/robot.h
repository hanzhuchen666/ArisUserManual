#ifndef ROBOT_H_
#define ROBOT_H_

#include <memory>
#include <aris.hpp>

namespace robot
{
    /**
     * @brief 单电机梯形曲线位置驱动，驱动单个电机在一个梯形曲线周期内走一段距离
     */
    class TcurveDrive :public aris::core::CloneObject<TcurveDrive,aris::plan::Plan>
    {

    public:
        /**
         * @brief 实时线程前的预处理
         */
        auto virtual prepareNrt()->void;
        /**
         * @brief 实时线程，每ms运行一轮
         * @return 当return值为0时，该线程结束
         */
        auto virtual executeRT()->int;
        /**
         * @brief 线程结束后处理
         */
        auto virtual collectNrt()->void;

        /**
         * @brief 析构函数
         */
        virtual ~TcurveDrive();
        /**
         * @brief 构造函数
         * @param name是指令名称，此处传入的值为默认值，实际没有起作用
         */
        explicit TcurveDrive(const std::string &name = "motor_drive");

    private:
        /**
         * @brief cef_成员变量，梯形曲线系数
         */
        double cef_;
    };


    /**
     * @brief 单电机正弦往返运动
     */
    class MoveJS : public aris::core::CloneObject<MoveJS, aris::plan::Plan>
      {
      public:

          auto virtual prepareNrt()->void;
          auto virtual executeRT()->int;
          auto virtual collectNrt()->void;

          explicit MoveJS(const std::string &name = "MoveJS_plan");

      };

    /**
     * @brief 单电机速度模式正弦运动
     */
    class VelDrive : public aris::core::CloneObject<VelDrive,aris::plan::Plan>
    {
     public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~VelDrive();
        explicit VelDrive(const std::string &name = "vel_drive");

     private:
        /**
         * @brief cef_成员变量，正弦函数幅值
         */
        double cef_;
    };


    auto createControllerMotor()->std::unique_ptr<aris::control::Controller>;
    auto createPlanMotor()->std::unique_ptr<aris::plan::PlanRoot>;
}

#endif
