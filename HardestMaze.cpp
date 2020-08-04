// C++11
#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <map>
#include <sstream>
#include <vector>
#include <set>
#include <string>

using namespace std;

class HardestMaze {
public:
  vector<char> findSolution(int N, int R, int T, vector< pair<int, int> > Starts, vector< vector< pair<int, int> > > Targets)
  {           
    vector<char> grid(N*N);
    for (int i=0; i<N*N; i++) grid[i]='.';
    return grid;
  }
};

int main() {
  HardestMaze prog;
  int N;
  int R;
  int T;
  int row, col;
  vector< pair<int, int> > Starts;
  vector< vector< pair<int, int> > > Targets;

  cin >> N;
  cin >> R;
  cin >> T;

  for (int i=0; i<R; i++)
  {
    cin >> row >> col;
    Starts.push_back(make_pair(row, col));

    vector< pair<int, int> > targ;
    for (int k=0; k<T; k++)
    {
      cin >> row >> col;
      targ.push_back(make_pair(row, col));
    }
    Targets.push_back(targ);
  }  
  
  vector<char> ret = prog.findSolution(N, R, T, Starts, Targets);
  cout << ret.size() << endl;
  for (int i = 0; i < (int)ret.size(); ++i)
      cout << ret[i] << endl;
  cout.flush();
}