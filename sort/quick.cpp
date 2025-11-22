#include <iostream>
using namespace std;
int a[100000];
void kuaisu(int a[], int left, int right) {
  int p = a[left];
  int z = left;
  int y = right;
  if (left >= right) {
    return;
  }
  while (left < right) {
    while (a[right] >= p && left < right) {
      right--;
    }
    while (a[left] <= p && left < right) {
      left++;
    }
    swap(a[left], a[right]);
  }
  swap(a[z], a[left]);
  kuaisu(a, z, left - 1);
  kuaisu(a, right + 1, y);
}
int main() {
  int n;
  cin >> n;
  for (int i = 0; i < n; i++) {
    cin >> a[i];
  }
  kuaisu(a, 0, n - 1);
  for (int i = 0; i < n; i++) {
    cout << a[i] << " ";
  }
}
