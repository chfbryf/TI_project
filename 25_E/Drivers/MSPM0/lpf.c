 #include "lpf.h"

//
// @简介：初始化一阶低通滤波器
//                                1
//        低通滤波器的公式为 ----------
//                            Tf*s + 1
// @参数：Tf - 低通滤波器的时间常数
//
void LPF_Init(LPF_TypeDef *Lpf, float Tf)
{
	Lpf->Tf = Tf;

	Lpf->LastTime = 0xffffffffffffffff; // 0xffffffff表示低通滤波器从未计算过
}

//
// @简介：计算低通滤波器的输出
// @参数：Lpf - 低通滤波器
// @参数：Input - 输入
// @参数：now - 当前时间，单位us
// @返回值：输入信号经由低通滤波器滤波后的结果
//
float LPF_Calc(LPF_TypeDef *Lpf, float Input, uint64_t now)
{
	float output;

	if(Lpf->LastTime == 0xffffffffffffffff)
	{
		output = Input; // 如果是第一次运算，那么将输入值直接输出出去
	}
	else
	{
		//          ------
		//  r(t)   |  1   |  c(t)
		// --- --> | ---- | ------>
		//  - ^    | Tf*s |    |
		//    |     ------     |
		//     ----------------
		//
		// 由框图可知 c(t) = c(t) + (r(t) - c(t)) * Δt / Tf
		float dt = (now - Lpf->LastTime) * 1e-6; // Δt
		output = Lpf->LastOutput + (Input - Lpf->LastOutput) / Lpf->Tf *dt;
	}

	Lpf->LastTime = now;
	Lpf->LastOutput = output;

	return output;
}
