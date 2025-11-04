#include<iostream>
#include<string>
using namespace std;
typedef unsigned int p;
class Solution{
	public:
		bool isAnagram(string s,string t){
			int hash[26]={0};
			for (p i=0;i<s.size();i++){
				hash[s[i]-'a']++;
			}
			for (p i=0;i<t.size();i++){
				hash[t[i]-'a']--;
			}
			for (int i=0;i<26;i++){
				if(hash[i]!=0){
					return false;
				}
			}
			return true;
		}
}; 
int main(){
	string m,n;
	cin>> m >> n;
	Solution k;
	cout <<(k.isAnagram(m,n) ? "Yes" : "No");
}
