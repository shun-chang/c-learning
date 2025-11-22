#include <iostream>
using namespace std;
int a[100000];
int b[100000];
void merge(int left, int right, int center) {
  int i = left;
  int j = center + 1;
  int k = 0;
  while (i <= center && j <= right) {
    if (a[i] <= a[j]) {
      b[k++] = a[i++];
    } else {
      b[k++] = a[j++];
    }
  }
  while (i <= center) {
    b[k++] = a[i++];
  }
  while (j <= right) {
    b[k++] = a[j++];
  }
  for (int m = left, n = 0; m <= right; m++) {
    a[m] = b[n++];
  }
}

void mergesort(int left, int right) {
  int center = (left + right) / 2;
  if (left == right) {
    return;
  }
  mergesort(left, center);
  mergesort(center + 1, right);
  merge(left, right, center);
}

int main() {
  int n;
  cin >> n;
  for (int i = 0; i < n; i++) {
    cin >> a[i];
  }
  mergesort(0, n - 1);
  for (int i = 0; i < n; i++) {
    cout << a[i] << " ";
  }
}