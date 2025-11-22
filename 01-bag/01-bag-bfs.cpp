#include <algorithm>
#include <iostream>
#include <queue>

using namespace std;

int v[10000];
int w[10000];
int n, m, sumv;
int best = 0;

struct node {
  int cp;    // 已装入背包的物品价值
  double up; // 价值上界
  int rw;    // 背包剩余容量
  int id;    // 物品序号
  node() {}
  node(int _cp, double _up, int _rw, int _id)
      : cp(_cp), up(_up), rw(_rw), id(_id) {}
};

struct goods {
  int id;
  double d; // 单位质量价值
} a[100000];

bool cmp(goods a, goods b) { // 按照物品单位质量价值从大到小排序
  return a.d > b.d;
}

bool operator<(const node &a, const node &b) { // 队列优先级，up越大优先级越高
  return a.up < b.up;
}

double Bound(node z) { // 计算节点z的价值上界
  int t = z.id;        // 背包序号
  int left = z.rw;     // 背包剩余容量
  double brp = 0.0;    // 背包剩余容量可以装入的物品最大价值
  while (t <= n && w[a[t].id] < left) {
    left -= w[a[t].id];
    brp += v[a[t].id];
    t++;
  }
  if (t <= n) {
    brp += 1.0 * v[a[t].id] / w[a[t].id] * left;
  }
  return z.cp + brp;
}

int priorbfs() {               // 优先队列式分支限界法
  priority_queue<node> q;      // 创建一个优先队列
  double tup;                  // 上界
  q.push(node(0, sumv, m, 1)); // 初始化，将根加入优先队列
  while (!q.empty()) {
    node cur, lc, rc; // 当前节点，左孩子，右孩子
    cur = q.top();
    q.pop();
    int t = cur.id; // 当前处理的物品序号
    if (t > n || cur.rw == 0)
      continue;
    if (cur.up < best)
      continue;
    int tt = a[t].id;
    if (cur.rw >= w[tt]) {
      lc.cp = cur.cp + v[tt];
      lc.rw = cur.rw - w[tt];
      lc.id = t + 1;
      tup = Bound(lc);
      lc.up = tup;
      if (lc.cp > best) {
        best = lc.cp;
      }
      q.push(lc);
    }
    rc.cp = cur.cp;
    rc.rw = cur.rw;
    rc.id = t + 1;
    tup = Bound(rc);
    if (tup > best) {
      rc = node(rc.cp, tup, rc.rw, rc.id);
      q.push(rc);
    }
  }
  return best;
}

int main() {
  cin >> n >> m;
  for (int i = 1; i <= n; i++) {
    cin >> w[i] >> v[i];
    sumv += v[i];
    a[i].id = i;                // 记录原始下标
    a[i].d = 1.0 * v[i] / w[i]; // 计算单位质量价值
  }
  sort(a, a + n + 1, cmp);
  priorbfs();
  cout << best;
}
