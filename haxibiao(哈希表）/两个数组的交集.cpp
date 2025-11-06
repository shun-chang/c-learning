#include<bits/stdc++.h>
using namespace std;
class Solution{
	public:
		vector<int> intersection(vector<int> num1,vector<int> num2){
			unordered_set<int> result;
			unordered_set<int> nnum1(num1.begin(),num1.end());
			for(unsigned i=0;i<num2.size();i++){
				if(nnum1.find(num2[i])!=nnum1.end()){
					result.insert(num2[i]);
				}
			}
			return vector<int> (result.begin(),result.end());
		}
};
vector<int> read(){
	string line;
	vector<int> num;
	getline(cin,line);
	//读取回车，以便结束读入，并进行下一次读入 
	if(line.empty()){
		return num;
	}
	stringstream ss(line);//把字符串line封装成"流"
	// 以便像操作文件 / 控制台流（cin/cout）一样
	//通过 >> 或 << 运算符拆分 / 拼接字符串
	int p;
	while(ss>>p){
		num.push_back(p);
	}
	return num;
}
int main(){
	vector<int> num1;
	vector<int> num2;
	Solution p;
	vector<int> op;
	op=p.intersection(num1,num2);
	for(int po:op){
		cout<<po<<" ";
	}
} 
