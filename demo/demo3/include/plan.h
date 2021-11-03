#ifndef PLAN_H_
#define PLAN_H_






/**
 * @brief 生成0->1的梯形曲线。可根据输入的加速度和速度判断曲线为梯形还是三角形
 * 
 */
class TCurve
{
private:
	double Tc_; //!<生成曲线总共需要的时间，由输入的加速度和速度计算
	double v_; //!< 速度，由用户输入，构造函数初始化
	double a_; //!< 加速度，由用户输入，构造函数初始化
	double ta_; //!<加速段所需的时间，由输入的速度和加速度计算得到

public:
/**
 * @brief 
 * 
 */

	/**
	 * @brief 
	 * 
	 * @param count 
	 * @return double 
	 */
	auto getTCurve(int count)->double;
	/**
	 * @brief Get the Curve Param object
	 * 
	 */
	auto getCurveParam()->void;
	/**
	 * @brief Get the Tc object
	 * 
	 * @return double 
	 */
	auto getTc()->double { return Tc_; };
	/**
	 * @brief Construct a new TCurve object
	 * 
	 * @param a 
	 * @param v 
	 */
	TCurve(double a, double v) { a_ = a; v_ = v; }
	~TCurve() {}
};

#endif



