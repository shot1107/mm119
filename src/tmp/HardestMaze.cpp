#include <bits/stdc++.h>
using namespace std;

#define REP(i, n) for ( int i = 0; i < (n); i++ )
const int dx[4] = {0, 1, 0, -1}, dy[4] = {-1, 0, 1, 0};
const int INF = 1e9;
struct Pt {
  int x, y;
  Pt() {}
  Pt(int x, int y): x(x), y(y) {}

};

const int64_t CYCLES_PER_SEC = 2800000000;
const double TIMELIMIT = 2.95;
struct Timer {
  int64_t start;
  Timer() { reset(); }
  void reset() { start = getCycle(); }
  void plus(double a) { start -= (a * CYCLES_PER_SEC); }
  inline double get() { return (double) (getCycle() - start) / CYCLES_PER_SEC; }
  inline int64_t getCycle() {
    uint32_t low, high;
    __asm__ volatile ("rdtsc" : "=a" (low), "=d" (high));
    return ((int64_t) low) | ((int64_t) high << 32);
  }
};
class XorShift {
 public:
  unsigned int x, y, z, w;
  double nL[65536];

  XorShift() {
    init();
  }

  void init() {
    x = 314159265;
    y = 358979323;
    z = 846264338;
    w = 327950288;
    double n = 1 / (double) (2 * 65536);
    for (int i = 0; i < 65536; i++) {
      nL[i] = log(((double) i / 65536) + n);
    }
  }

  inline unsigned int next() {
    unsigned int t = x ^x << 11;
    x = y;
    y = z;
    z = w;
    return w = w ^ w >> 19 ^ t ^ t >> 8;
  }

  inline double nextLog() {
    return nL[next() & 0xFFFF];
  }

  inline int nextInt(int m) {
    return (int) (next() % m);
  }

  int nextInt(int min, int max) {
    return min + nextInt(max - min + 1);
  }

  inline double nextDouble() {
    return (double) next() / ((long long) 1 << 32);
  }

};
XorShift rnd;

const int MAX_N = 40;
const int MAX_R = 6;
const int MAX_T = 6;
int N; // 盤面の幅
int R; // ロボットの数
int T; // target(ロボットが訪れないといけない地点)の数
vector<vector<vector<int> > > grids; // robot : T  target : i (0 <= i < T)  else -1
vector<vector<bool> > all_grid; // 何かしら置かれていたらtrue
vector<Pt> starts;
vector<vector<Pt> > targets;


struct State {
  int cost;
  vector<vector<bool> >wall;
  State() {
    cost = 0;
    this->wall.resize(N, vector<bool>(N, false)); // 壁が全くない状態で初期化
  }

  State(int cost, vector<vector<bool> > wall) :
    cost(cost), wall(wall) {}

  bool operator<(const State &another) const {
    return cost < another.cost;
  }

  bool operator>(const State &another) const {
    return cost > another.cost;
  }
};

void input() {
  cin >> N >> R >> T;
  grids.resize(R, vector<vector<int> >(N, vector<int>(N, -1)));
  all_grid.resize(N, vector<bool>(N, false));
  starts.resize(R);
  targets.resize(R);

  REP(i, R) {
    cin >> starts[i].y >> starts[i].x;
    grids[i][starts[i].y][starts[i].x] = T;
    all_grid[starts[i].y][starts[i].x] = true;
  }
  REP(i, R) {
    targets[i].resize(T);
    REP(j, T) {
      cin >> targets[i][j].y >> targets[i][j].x;
      grids[i][targets[i][j].y][targets[i][j].x] = j;
      all_grid[targets[i][j].y][targets[i][j].x] = true;
    }
  }

}

bool is_contained(int x, int y) {
  return x >= 0 && y >= 0 && x < N && y < N;
}

int calc_dist(Pt start, Pt target, vector<vector<bool> > &wall) {
  int dist[MAX_N][MAX_N];
  fill_n(*dist, MAX_N*MAX_N, -1);
  queue<Pt> Q;
  Q.push(start);
  dist[start.y][start.x] = 0;
  while (!Q.empty()) {
    Pt p = Q.front();
    Q.pop();
    int x = p.x, y = p.y;
    if (x == target.x && y == target.y) return dist[y][x];
    REP(i, 4) {
      int nx = x + dx[i], ny = y + dy[i];
      if (!is_contained(nx, ny)) continue;
      if (wall[ny][nx] || dist[ny][nx] >= 0) continue;
      dist[ny][nx] = dist[y][x]+1;
      Q.push(Pt(nx, ny));
    }
  }

  return INF;
}

// ロボットrについて壁が置かれているマスの状態Wallであるときの最短コスト
int calc_cost(int r, vector<vector<bool> > &wall) {
  int dp[MAX_T][1 << MAX_T];
  int dist[MAX_T][MAX_T];
  fill_n(*dp,  MAX_T * (1 <<MAX_T), -1);
  fill_n(*dist, MAX_T*MAX_T, INF);
  REP(i, T) {
    dist[i][i] = 0;
    for ( int j = i+1; j < T; j++ ) dist[i][j] = dist[j][i] = calc_dist(targets[r][i], targets[r][j], wall);
  }
  function<int(int, int)> dfs = [&](int v, int bit) {
    if (bit == (1 << T) - 1) return 0;
    if (dp[v][bit] >= 0) return dp[v][bit];
    int ret = INF;
    REP(i, T) {
      if ( bit & (1<<i) ) continue;
      ret = min(ret, dfs(i, bit|(1<<i))+dist[v][i]);
    }

    return dp[v][bit] = ret;
  };

  int ret = INF;
  REP(i, T) {
    ret = min(ret, dfs(i, (1<<i))+calc_dist(starts[r], targets[r][i], wall));
  }
  return ret;
}

vector<State> beam_search() {
  priority_queue<State> Q;
  Q.push(State());

  REP(_, 100) {
    priority_queue<State> nQ;
    vector<Pt> put_xy;
    const int num_put = 30;
    REP(i, num_put) { // 壁を置く候補を１０個ランダムに決める
      int xy = rnd.nextInt(N*N);
      put_xy.emplace_back(Pt(xy%N, xy/N));
    }
    while ( !Q.empty() ) {
      nQ.push(Q.top());
      State now = Q.top(); Q.pop();
      vector<vector<bool> > nwall = now.wall;
      REP(i, num_put) {
        int x = put_xy[i].x, y = put_xy[i].y;
        if ( !all_grid[y][x] ) nwall[y][x] = true;
      }
      int ncost = 0;
      REP(i, R) {
        ncost += calc_cost(i, nwall);
      }
      if ( ncost < INF ) nQ.push(State(ncost, nwall));
    }

    REP(i, min(10, (int)nQ.size())) {
      Q.push(nQ.top()); nQ.pop();
    }
  }

  vector<State> ret;
  REP(i, min(3, (int)Q.size())) {
    ret.push_back(Q.top()); Q.pop();
  }

  return ret;
}

void output(State &state) {
  cout << N*N << endl;
  REP(i, N) REP(j, N) {
    if ( state.wall[i][j] ) cout << '#' << endl;
    else cout << '.' << endl;
  }
}

void solve() {
  input();
  vector<State> init_state = beam_search();
  output(init_state[0]);
}

signed main() {
  cin.tie(0);
  ios_base::sync_with_stdio(0);
  cout << fixed << setprecision(12);

  solve();

  return 0;
}
