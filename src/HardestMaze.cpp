#include <bits/stdc++.h>
using namespace std;
#define int long long
#define REP(i, n) for ( int i = 0; i < (n); i++ )
const int dx[4] = {0, 1, 0, -1}, dy[4] = {-1, 0, 1, 0};
const int INF = 1e15;
struct Pt {
  int x, y;
  Pt() {}
  Pt(int x, int y) : x(x), y(y) {}

};

const int64_t CYCLES_PER_SEC = 2800000000;
const double TIMELIMIT = 3.0;
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
  long long cost;
  vector<vector<bool> > wall;
  State() {
    cost = 0;
    this->wall.resize(N, vector<bool>(N, false)); // 壁が全くない状態で初期化
  }

  State(long long cost, vector<vector<bool> > wall) :
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
  fill_n(*dist, MAX_N * MAX_N, -1);
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
      dist[ny][nx] = dist[y][x] + 1;
      Q.push(Pt(nx, ny));
    }
  }

  return INF;
}

long long dp[MAX_T][1 << MAX_T];
long long dist[MAX_T][MAX_T];
int dfs(int v, int bit) {
  if (bit == (1 << T) - 1) return 0;
  if (dp[v][bit] >= 0) return dp[v][bit];
  long long ret = INF;
  REP(i, T) {
    if (bit & (1 << i)) continue;
    ret = min(ret, dfs(i, bit | (1 << i)) + dist[v][i]);
  }

  return dp[v][bit] = ret;
}

// ロボットrについて壁が置かれているマスの状態Wallであるときの最短コスト
long long calc_cost(int r, vector<vector<bool> > &wall) {
  fill_n(*dist, MAX_T * MAX_T, INF);
  fill_n(*dp, MAX_T * (1 << MAX_T), -1);
  REP(i, T) {
    dist[i][i] = 0;
    for (int j = i + 1; j < T; j++) {
      int tmp = calc_dist(targets[r][i], targets[r][j], wall);
      if (tmp >= INF) return INF;
      dist[i][j] = dist[j][i] = tmp;
    }
  }
  int ret = INF;
  REP(i, T) {
    ret = min(ret, dfs(i, (1 << i)) + calc_dist(starts[r], targets[r][i], wall));
  }
  return ret;
}

// 状態遷移
void modify (State& state) {
  vector<Pt> put_xy, erase_xy;
  int num_put = 0, num_erase = 0;
  if ( rnd.nextInt(10) <= 8 ) num_put = 1;
  if ( rnd.nextInt(10) <= 5 ) num_erase = 1;
  REP(i, num_put) { // 壁を置く候補を１０個ランダムに決める
    int xy = rnd.nextInt(N * N);
    put_xy.emplace_back(Pt(xy % N, xy / N));
  }
  REP(i, num_erase) { // 消す壁の候補を１０個ランダムに決める
    int xy = rnd.nextInt(N * N);
    erase_xy.emplace_back(Pt(xy % N, xy / N));
  }

  if ( num_erase == 0 && num_put == 0 ) return;
  bool flag = false;
  vector<vector<bool> > &nwall = state.wall;
  REP(i, num_put) {
    int x = put_xy[i].x, y = put_xy[i].y;
    if (!all_grid[y][x] && !nwall[y][x]) {
      nwall[y][x] = true;
      flag = true;
    }
  }
  REP(i, num_erase) {
    int x = erase_xy[i].x, y = erase_xy[i].y;
    if (all_grid[y][x] && nwall[y][x]) {
      nwall[y][x] = false;
      flag = true;
    }
  }
  if ( !flag ) return;
  long long ncost = 0;
  REP(i, R) {
    ncost += calc_cost(i, nwall);
  }
  if ( ncost >= INF ) ncost = -INF;
  state.cost = ncost;
}

// 状態のスコア計算
int calc_score (State& state) {
  return state.cost;
}

State simulated_annealing(State state) {
  double start_temp = 50, end_temp = 10; // 適当な値を入れる（後述）
  Timer timer;
  double start_time = timer.get(); // 開始時刻
  while (true) { // 時間の許す限り回す
    double now_time = timer.get(); // 現在時刻
    if (now_time - start_time > TIMELIMIT) break;

    State new_state = state;
    modify(new_state);
    int new_score = calc_score(new_state);
    int pre_score = calc_score(state);

    // 温度関数
    double temp = start_temp + (end_temp - start_temp) * (now_time-start_time) / TIMELIMIT;
    // 遷移確率関数(最大化の場合)
    double prob = exp((new_score-pre_score)/temp);

    if (prob > (rand()%INF)/(double)INF) { // 確率probで遷移する
      state = new_state;
    }
  }

  return state;
}

vector<State> beam_search(vector<State> states) {
  priority_queue<State> Q;
  for (State &i: states) Q.push(i);
  // Q.push(State());
  // cout << Q.size() << endl;
  REP(_, 800) {
    priority_queue<State> nQ;
    vector<Pt> put_xy, erase_xy;
    int num_put = 1, num_erase = rnd.nextInt(3);
    REP(i, num_put) { // 壁を置く候補を１０個ランダムに決める
      int xy = rnd.nextInt(N * N);
      put_xy.emplace_back(Pt(xy % N, xy / N));
    }
    REP(i, num_erase) { // 消す壁の候補を１０個ランダムに決める
      int xy = rnd.nextInt(N * N);
      erase_xy.emplace_back(Pt(xy % N, xy / N));
    }

    unordered_map<int, bool> used;
    while (!Q.empty()) {
      bool flag = false;
      nQ.push(Q.top());
      State now = Q.top();
      Q.pop();
      vector<vector<bool> > nwall = now.wall;
      REP(i, num_put) {
        int x = put_xy[i].x, y = put_xy[i].y;
        if (!all_grid[y][x] && !nwall[y][x]) {
          flag = true;
          nwall[y][x] = true;
        }
      }
      REP(i, num_erase) {
        int x = erase_xy[i].x, y = erase_xy[i].y;
        if (all_grid[y][x] && nwall[y][x]) {
          flag = true;
          nwall[y][x] = false;
        }
      }
      if (!flag) continue;
      long long ncost = 0;
      REP(i, R) {
        ncost += calc_cost(i, nwall);
      }
      if (ncost < INF && used.count(ncost) == 0 ) {
        nQ.push(State(ncost, nwall));
        used[ncost] = true;
      }
    }
    // cout << "nQ size " << nQ.size() << endl;
    int nQ_size = nQ.size();
    REP(i, min(5LL, nQ_size)) {
      Q.push(nQ.top());
      nQ.pop();
    }
  }
  vector<State> ret;
  REP(i, min(3LL, (int) Q.size())) {
    ret.push_back(Q.top());
    Q.pop();
  }

  return ret;
}

void output(State &state) {
  cout << N * N << endl;
  REP(i, N) REP(j, N) {
      if (state.wall[i][j]) cout << '#' << endl;
      else cout << '.' << endl;
    }
  // cout << state.cost << endl;
}

vector<State> create_initial_states() {
  vector<State> states;
  {
    State state;
    bool cnt = false;
    REP(i, N) {
      if (i % 2) continue;
      bool exist = false;
      REP(j, N) {
        if (all_grid[i][j]) exist = true;
        else state.wall[i][j] = true;
      }
      if (exist) continue;
      if (cnt) {
        state.wall[i][0] = false;
      } else {
        state.wall[i][N - 1] = false;
      }
      cnt = !cnt;
    }
    long long ncost = 0;
    REP(i, R) {
      ncost += calc_cost(i, state.wall);
    }
    state.cost = ncost;
    states.emplace_back(state);
  }
  {
    State state;
    bool cnt = false;
    REP(i, N) {
      if (i % 2 == 0) continue;
      bool exist = false;
      REP(j, N) {
        if (all_grid[i][j]) exist = true;
        else state.wall[i][j] = true;
      }
      if (exist) continue;
      if (cnt) {
        state.wall[i][0] = false;
      } else {
        state.wall[i][N - 1] = false;
      }
      cnt = !cnt;
    }
    long long ncost = 0;
    REP(i, R) {
      ncost += calc_cost(i, state.wall);
    }
    state.cost = ncost;
    states.emplace_back(state);
  }
  {
    State state;
    bool cnt = false;
    REP(j, N) {
      if (j % 2) continue;
      bool exist = false;
      REP(i, N) {
        if (all_grid[i][j]) exist = true;
        else state.wall[i][j] = true;
      }
      if (exist) continue;
      if (cnt) {
        state.wall[0][j] = false;
      } else {
        state.wall[N - 1][j] = false;
      }
      cnt = !cnt;
    }
    long long ncost = 0;
    REP(i, R) {
      ncost += calc_cost(i, state.wall);
    }
    state.cost = ncost;
    states.emplace_back(state);
    /* REP(i, N) {
      REP(j, N) {
        cout << state.wall[i][j] << " ";
      } cout << endl;
    } */
  }
  {
    State state;
    bool cnt = false;
    REP(j, N) {
      if (j % 2 == 0) continue;
      bool exist = false;
      REP(i, N) {
        if (all_grid[i][j]) exist = true;
        else state.wall[i][j] = true;
      }
      if (exist) continue;
      if (cnt) {
        state.wall[0][j] = false;
      } else {
        state.wall[N - 1][j] = false;
      }
      cnt = !cnt;
    }
    long long ncost = 0;
    REP(i, R) {
      ncost += calc_cost(i, state.wall);
    }
    state.cost = ncost;
    states.emplace_back(state);
  }

  return states;
}

void solve() {
  input();
  vector<State> states = create_initial_states();
  // states = beam_search(states);
  State ans = simulated_annealing(states[0]);
  for ( int i = 1; i < (int)states.size(); i++ ) {
    ans = max(ans, simulated_annealing(states[i]));
  }
  output(ans);
}

signed main() {
  cin.tie(0);
  ios_base::sync_with_stdio(0);
  cout << fixed << setprecision(12);

  solve();

  return 0;
}
