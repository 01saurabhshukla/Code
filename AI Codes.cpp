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
