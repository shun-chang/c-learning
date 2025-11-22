#include <iostream>
#include <string.h>
using namespace std;
int W, n;
int cp = 0, w = 0;
int best = 0;
struct node {
  int w, v;
} a[10000];

int can(int t) {
  int p = 0;
  int left = 0;
  for (int i = t; i <= n; i++) {
    left += a[i].v;
  }
  if (cp + left > best) {
    p = 1;
  }
  return p;
}

void max_value(int t) {
  if (t == n + 1) {
    if (cp > best) {
      best = cp;
    }
  }
  if (w + a[t].w <= W && can(t)) {
    w += a[t].w;
    cp += a[t].v;
    max_value(t + 1);
    w -= a[t].w;
    cp -= a[t].v;
  }
  max_value(t + 1);
}

int main() {
  cin >> n >> W;
  for (int i = 1; i <= n; i++) {
    cin >> a[i].w >> a[i].v;
  }
  max_value(1);
  cout << best;
}
