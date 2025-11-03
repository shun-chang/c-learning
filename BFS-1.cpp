#include<bits/stdc++.h>
using namespace std;
struct p{
	int x,y;
	int step;
};
int main(){
	int m,n,d;
	cin >> m >> n >> d;
	p s,e;
	s.x=0;
	s.y=0;
	int k=0;
	s.step=0;
	queue<p> b;
	b.push(s);
	int x,y,step;
	while(!b.empty()){
		if(k==1) break;
		s=b.front(),b.pop();
		step=s.step;
		for (int i=0;i<6;i++){
			x=s.x;
			y=s.y;
			switch(i){
				case(0): x=m;
				break;
				case(1): x=0;
				break;
				case(2): y=n;
				break;
				case(3): y=0;
				break;
				case(4):
					if(x+y>n){
						x=x+y-n;
						y=n;
					}
					else{
						y=x+y;
						x=0;
					}
					break;
				case(5):
					if(x+y>m){
						y=x+y-m;
						x=m;
					}
					else{
						x=x+y;
						y=0;
					}
					break;
			}
			if(x==d||y==d){
				k=1;
				break;
			}
			else{
				e.x=x;
				e.y=y;
				e.step=step+1;
				b.push(e);
			}
		}
	}
	cout << step+1;
} 
