BFS

#include <bits/stdc++.h>

using namespace std;
class Solution {
  public:
    vector < int > bfsOfGraph(int V, vector < int > adj[]) {
      vector < int > bfs;
      vector < int > vis(V, 0);
      queue < int > q;
      q.push(0);
      vis[0] = 1;
      while (!q.empty()) {
        int node = q.front();
        q.pop();
        bfs.push_back(node);

        for (auto it: adj[node]) {
          if (!vis[it]) {
            q.push(it);
            vis[it] = 1;
          }
        }
      }

      return bfs;
    }
};

void addEdge(vector < int > adj[], int u, int v) {
  adj[u].push_back(v);
  adj[v].push_back(u);
}

void printAns(vector < int > & ans) {
  for (int i = 0; i < ans.size(); i++) {
    cout << ans[i] << " ";
  }
}
int main() {
  vector < int > adj[5];

  addEdge(adj, 0, 1);
  addEdge(adj, 0, 2);
  addEdge(adj, 0, 3);
  addEdge(adj, 2, 4);

  Solution obj;
  vector < int > ans = obj.bfsOfGraph(5, adj);
  printAns(ans);
  cout << endl;

  return 0;
}



DFS

#include <bits/stdc++.h>
using namespace std;

class Solution {
  private: 
    void dfs(int node, vector<int> adj[], int vis[], vector<int> &ls) {
        vis[node] = 1; 
        ls.push_back(node); 
        // traverse all its neighbours
        for(auto it : adj[node]) {
            // if the neighbour is not visited
            if(!vis[it]) {
                dfs(it, adj, vis, ls); 
            }
        }
    }
  public:
    // Function to return a list containing the DFS traversal of the graph.
    vector<int> dfsOfGraph(int V, vector<int> adj[]) {
        int vis[V] = {0}; 
        int start = 0;
        // create a list to store dfs
        vector<int> ls; 
        // call dfs for starting node
        dfs(start, adj, vis, ls); 
        return ls; 
    }
};

void addEdge(vector <int> adj[], int u, int v) {
    adj[u].push_back(v);
    adj[v].push_back(u);
}

void printAns(vector <int> &ans) {
    for (int i = 0; i < ans.size(); i++) {
        cout << ans[i] << " ";
    }
}

int main() 
{
    vector <int> adj[5];
    
    addEdge(adj, 0, 2);
    addEdge(adj, 2, 4);
    addEdge(adj, 0, 1);
    addEdge(adj, 0, 3);

    Solution obj;
    vector <int> ans = obj.dfsOfGraph(5, adj);
    printAns(ans);

    return 0;
}


MINIMAX
#include <iostream>
using namespace std;

int minmax(bool maximizer, int finalStates[], int l, int r)
{
    if (l == r)
    {
        return finalStates[l];
    }

    int mid = (l + r) / 2;

    if (Maximizer)
    {
        return max(minmax(false, finalStates, l, mid), minmax(false, finalStates, mid + 1, r));
    }

    return min(minmax(true, finalStates, l, mid), minmax(true, finalStates, mid + 1, r));
}

int main()
{
    int n;
    cout << "Enter the number of final states N, where ( N = 2^x ) :";
    cin >> n;
    cout << "Enter the final states : \n";
    int finalStates[n];
    for (int i = 0; i < n; i++)
    {
        cin >> finalStates[i];
    }

    int ans = minmax(true, finalStates, 0, n - 1);

    cout << " Best The Maximizer can get : " << ans << endl;

    return 0;
}




A* Code
def aStarAlgo(start_node, stop_node):
open_set = set(start_node)
closed_set = set()
g = {}
parents = {}
g[start_node] = 0
parents[start_node] = start_node
while len(open_set) > 0:
n = None
for v in open_set:
if n == None or g[v] + heuristic(v) < g[n] + heuristic(n):
n = v
if n == stop_node or Graph_nodes[n] == None:
pass
else:
for (m, weight) in get_neighbors(n):
if m not in open_set and m not in closed_set:
open_set.add(m)
parents[m] = n
g[m] = g[n] + weight
else:
if g[m] > g[n] + weight:
g[m] = g[n] + weight
parents[m] = n
if m in closed_set:
closed_set.remove(m)
open_set.add(m)
if n == None:
print('Path does not exist!')
return None
if n == stop_node:
path = []
while parents[n] != n:
path.append(n)
n = parents[n]
path.append(start_node)
path.reverse()
print('Path found: {}'.format(path))
return path
open_set.remove(n)
closed_set.add(n)
print('Path does not exist!')
return None
def get_neighbors(v):
if v in Graph_nodes:
return Graph_nodes[v]
else:
return None
def heuristic(n):
H_dist = {
'A': 11,
'B': 6,
'C': 5,
'D': 7,
'E': 3,
'F': 6,
'G': 5,
'H': 3,
'I': 1,
'J': 0
}
return H_dist[n]
Graph_nodes = {
'A': [('B', 6), ('F', 3)],
'B': [('A', 6), ('C', 3), ('D', 2)],
'C': [('B', 3), ('D', 1), ('E', 5)],
'D': [('B', 2), ('C', 1), ('E', 8)],
'E': [('C', 5), ('D', 8), ('I', 5), ('J', 5)],
'F': [('A', 3), ('G', 1), ('H', 7)],
'G': [('F', 1), ('I', 3)],
'H': [('F', 7), ('I', 2)],
'I': [('E', 5), ('G', 3), ('H', 2), ('J', 3)],
}
aStarAlgo('A', 'J')
