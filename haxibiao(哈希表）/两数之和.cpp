#include <vector>
#include <unordered_map>
#include <sstream>
#include <string>
#include <iostream>
using namespace std;
class Solution{
	public:
		vector<int> f(vector<int> num,int target){
			unordered_map<int,int> result;
			for(size_t i=0;i<num.size();i++){
				auto turn=result.find(target-num[i]);
				if(turn!=result.end()){
					return {turn->second,(int)i};
				}
				else{
					//用pair把num[i]和i变成一个整体 
					result.insert(pair<int,int> (num[i],i));
					//result.insert({num[i],i});
					//result[num[i]]=i;
					//两种等价形式 
				}
			}
			
			return {};
		}
};
vector<int> read(){
	string m;
	vector<int> num;
	getline(cin,m);
	if(m.empty()){
		return num;
	}
	stringstream op(m);
	int p;
	while(op>>p){
		num.push_back(p);
	}
	return num;
}
int main(){
	vector<int> num=read();
	int target;
	cin>>target;
	Solution p;
	vector<int> op;
	op=p.f(num,target);
	if(op.empty()) cout<<"none"<<endl;
	else{ 
		for(int pp : op){
			cout<<pp<<" ";
		}
	}
}
