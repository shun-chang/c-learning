#include<bits/stdc++.h>
using namespace std;
int main(){
	srand((unsigned int ) time(nullptr));//初始化随季节种子，且只需要执行一次，以避免每次运行结果相同
	double num;
	num = (double) rand()/RAND_MAX;
  	//rand()生成[0,RAND_MAX]的随机整数
	cout << num;



	default_random_engine int_engine;//应只初始化一次，防止出现重复，及不要出现在函数中，可加static写在main函数外
	//生成原始伪随机数序列（相当于“随机源”），是生成随机数的基础引擎
	uniform_int_distribution<unsigned int> udistribution(100000,999999);//应只初始化一次，防止出现重复
	//将引擎生成的原始随机数缩放调整为指定范围的无符号整数
	//int为生成随机整数，且范围为左闭右闭
	unsigned int urandom = udistribution(int_engine);
	cout << urandom;
	uniform_int_distribution<int> distribution(-999999,-100000);
	int random = distribution(int_engine);
	cout << random;
	default_random_engine real_engine;
	uniform_real_distribution<double> ddistribution(10000.1,99999.9);
	//real为生成随机浮点数，且范围为左闭右开
	double dis = ddistribution(real_engine);
	cout << dis;


	random_device q;
	default_random_engine p(g);
	uniform_int_distribution<int> distribution2(-999999,100000);
	cout << distribution2(p);
	//先种种子，再生成随机数，这样更随机



	random_device rd;
	//用于获得随机数引擎种子
	mt19937 gen(rd());
	//以rd（）播种的标准 mersenne_twister_engine
	uniform_real_distribution<double> dis(10000,99999);
	for (int i=0;i<10;i++){
		cout << dis(gen) << " ";
	}
	return 0;
}



