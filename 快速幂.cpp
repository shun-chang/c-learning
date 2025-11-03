#include<bits/stdc++.h>
using namespace std;
typedef long long p;
p b=1;
p f(p m,p n,p a){
	while(n>0){
		if(n & 1){
			b=(m*b)%a;
			n>>=1;
			m=(m*m)%a;
		}
	}
	return b;
}
int main(){
	int a,m,n;
	cin >> m >> n >> a;
	f(m,n,a);
	cout << b;
}
//求m的n次方模a的值，该算法的时间复杂度较低
