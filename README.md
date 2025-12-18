# 🗺️ Graph Traversal & Pathfinding System

<div align="center">

### 🎯 Smart City Navigation Using Functional Programming

*Find the shortest route between any two locations in a city network*

</div>

---

## 👥 Team Members

<table>
<tr>
<th>Name</th>
<th>Registration Number</th>
</tr>
<tr><td>K.D.L. Navoda</td><td>EG/2020/4088</td></tr>
<tr><td>D.M.I. Umayangana</td><td>EG/2020/4248</td></tr>
<tr><td>H.W.S.H. Vidusanka</td><td>EG/2020/4251</td></tr>
<tr><td>D.I. Vihan</td><td>EG/2020/4323</td></tr>
</table>


---

## 🌟 What is This Project?

Imagine you're in a city with **10 different locations** (like City Center, Airport, University, Beach) connected by **34 roads**. You want to find:

- 🛣️ **The shortest route** from one place to another
- ⚡ **The fastest way** to reach your destination
- 🔄 **Which places are reachable** from your current location
- 📊 **Compare different pathfinding methods**

This project solves exactly that using **4 powerful algorithms** implemented in pure functional Haskell!

### 🎯 Real-World Applications

This same technology is used in:
- 🚗 **GPS Navigation** (Google Maps, Waze)
- 📦 **Delivery Services** (Amazon, UPS route optimization)
- 🚌 **Public Transport** (Bus/train route planning)
- 🚑 **Emergency Services** (Fastest ambulance routes)
- 🎮 **Video Games** (AI pathfinding)

---

## 🏙️ City Network Map

Our sample city has these locations:

```
         J (Mountain View)
         |
    E -- B -- F (Airport)
    |    |    |
    H -- A -- C -- I (Beach Resort)
         |    |
         D    G (Industrial Zone)
    (South Port)

Legend:
A = City Center (Hub)      F = Airport
B = North Station          G = Industrial Zone  
C = East Market            H = University
D = South Port             I = Beach Resort
E = West Plaza             J = Mountain View
```

**Total:** 10 Locations | 34 Roads (bidirectional) | Each road has a distance

> 📊 **For detailed network visualization with all connections and distances, see [network_visualization.txt](network_visualization.txt)**

---

## 🚀 Quick Start Guide

### Step 1: Install Haskell

**Windows:**
```powershell
# Download and install from:
https://www.haskell.org/platform/

# OR use Chocolatey:
choco install ghc
```

**Verify Installation:**
```powershell
ghc --version    # Should show GHC version
ghci --version   # Should show GHCi version
```

### Step 2: Navigate to Project

```powershell
cd "e:\new projects\FP Group project"
```

### Step 3: Run the Program

**🎯 Easiest Method (Recommended):**
```powershell
runhaskell Main.hs  # Run immediately!
```

**⚡ Alternative Methods:**

*Compile First (faster execution):*
```powershell
ghc -O2 --make Main.hs -o PathFinder.exe
.\PathFinder.exe
```

*Interactive Mode (for testing):*
```powershell
ghci Main.hs
*Main> main
```

### Step 4: Enjoy! 🎉

The program will show an interactive menu - just follow the prompts!

---

## 💡 How to Use - Simple Examples

### Example 1: Find Route from City Center to Beach 🏖️

**What you do:**
```
1. Choose option: 1 (Find path)
2. Select algorithm: 3 (Dijkstra - finds shortest route)
3. From: A (City Center)
4. To: I (Beach Resort)
```

**What you get:**
```
[SUCCESS] Path found!
Route: A -> C -> I
Total cost: 11.7 km
Number of hops: 2 stops

Translation: Go from City Center to East Market, then to Beach Resort
Total distance: 11.7 kilometers, passing through 2 locations
```

### Example 2: Compare All Methods 🔬

**What you do:**
```
1. Choose option: 2 (Compare algorithms)
2. From: A (City Center)
3. To: F (Airport)
```

**What you get:**
```
+========================================+
|     ALGORITHM COMPARISON RESULTS       |
+========================================+

BFS (Breadth-First Search)
  Route: A -> B -> F
  Cost: 9.7 km | Hops: 2

DFS (Depth-First Search)  
  Route: A -> C -> F
  Cost: 11.0 km | Hops: 2

Dijkstra's Algorithm [BEST]
  Route: A -> B -> F
  Cost: 9.7 km | Hops: 2

A* Algorithm [FASTEST]
  Route: A -> B -> F
  Cost: 9.7 km | Hops: 2

Winner: Dijkstra & A* found optimal 9.7 km route!
```

### Example 3: Check Reachability 🔗

**What you do:**
```
1. Choose option: 3 (Connectivity)
2. Start from: A (City Center)
```

**What you get:**
```
Graph is CONNECTED
All locations are reachable from everywhere!

From City Center (A):
  - Can reach: 10/10 locations
  - Coverage: 100%

This means you can travel from City Center to 
any other location in the city!
```

---

## 🧠 The 4 Smart Algorithms Explained

### 1. 🌊 BFS (Breadth-First Search)

**What it does:** Explores all neighbors level by level  
**Best for:** Finding path with fewest stops  
**Speed:** ⚡ Fast (O(V+E))  
**Example:** Visit all nearby places first, then expand outward

```
You're at A, check B,C,D,H first
Then check their neighbors F,G,E
Keep expanding until you reach destination
```

### 2. 🏊 DFS (Depth-First Search)

**What it does:** Explores as deep as possible before backtracking  
**Best for:** Finding any path quickly  
**Speed:** ⚡ Fast (O(V+E))  
**Example:** Follow one route completely before trying another

```
You're at A, go to B
From B, go deep to F
From F, explore fully before returning
```

### 3. 🎯 Dijkstra's Algorithm

**What it does:** Finds the shortest distance considering road lengths  
**Best for:** Optimal route with minimum distance  
**Speed:** ⚡⚡ Moderate (O((V+E) log V))  
**Example:** Always pick the shortest available road

```
Start at A (distance: 0)
Check all roads from A:
  A→B: 5.5 km
  A→C: 7.2 km  ← Shorter, go here first
  A→D: 4.8 km  ← Even shorter!
Pick shortest, repeat until destination
```

### 4. ⭐ A* Algorithm

**What it does:** Like Dijkstra but also considers direction to goal  
**Best for:** Fastest pathfinding with smart guessing  
**Speed:** ⚡⚡⚡ Fast with heuristic  
**Example:** Uses GPS coordinates to estimate remaining distance

```
At each step:
  Distance traveled + Estimated remaining distance
Pick the most promising path first
Reaches goal faster than Dijkstra!
```

---

## 🎓 Functional Programming Concepts (Simply Explained)

### 1. 🔒 Pure Functions = No Surprises!

**Traditional programming:**
```
x = 5
result = calculate(x)  // Result might change!
x = 10                 // Someone changed x!
result = calculate(x)  // Different result 😱
```

**Functional programming:**
```haskell
calculate :: Int -> Int
calculate x = x * 2

result1 = calculate 5  -- Always 10
result2 = calculate 5  -- Always 10 (guaranteed!)
```

**Why it's great:** Same input → Same output (Always predictable!)

### 2. 🧊 Immutable Data = No Accidental Changes

**Traditional:**
```
graph = createGraph()
modifyGraph(graph)     // Oops! Original changed
// Original data is lost!
```

**Functional:**
```haskell
originalGraph = createGraph()
newGraph = modifyGraph originalGraph
-- Both originalGraph and newGraph exist!
-- Nothing was destroyed!
```

**Why it's great:** Can't accidentally break data

### 3. 🎭 Pattern Matching = Clear Logic

**Traditional:**
```
if (result.type == "found") {
    // do something
} else if (result.type == "not_found") {
    // do something else
} // Easy to miss cases!
```

**Functional:**
```haskell
case result of
    PathFound path    -> showPath path
    PathNotFound      -> showError
    InvalidNode node  -> showWarning node
-- Compiler checks you handled everything!
```

**Why it's great:** Compiler ensures you handle all cases

### 4. 🔄 Recursion = Natural Loops

**Traditional loop:**
```
for (i = 0; i < items.length; i++) {
    process(items[i]);
}
```

**Functional recursion:**
```haskell
processAll [] = "Done!"
processAll (first:rest) = 
    process first
    processAll rest  -- Call itself with remaining items
```

**Why it's great:** More natural for tree/graph problems

### 5. 🏗️ Higher-Order Functions = Reusable Patterns

**Without:**
```
result1 = []
for item in list:
    result1.append(double(item))

result2 = []
for item in list:
    result2.append(triple(item))
```

**With:**
```haskell
result1 = map double list    -- Apply double to all
result2 = map triple list    -- Apply triple to all
result3 = map anything list  -- Reusable pattern!
```

**Why it's great:** Write once, use everywhere

---

## 📂 Project Files Explained

```
📁 FP Group project/
│
├── 🚀 Main.hs                 👈 Start here! Main program
│   └── Controls menu, user interaction, coordinates everything
│
├── 📦 DataTypes.hs            👈 Data definitions
│   └── Defines Graph, Node, Edge, Path (like a blueprint)
│
├── 🧮 Processing.hs           👈 The brain! 
│   └── BFS, DFS, Dijkstra, A* algorithms
│
├── 💬 IOHandler.hs            👈 Communication
│   └── Handles user input, file reading, displaying results
│
├── 🔧 Utils.hs                👈 Helper tools
│   └── Common functions used by other files
│
├── 🗺️ city_network.txt       👈 City map data
│   └── Lists all locations and roads
│
├── 📊 network_visualization.txt  👈 Visual network map
│   └── ASCII diagram showing how nodes connect
│
└── 📖 README.md               👈 You are here!
    └── This helpful guide
```

### How Files Work Together:

```
User runs Main.hs
    ↓
Main.hs uses IOHandler.hs to load city_network.txt
    ↓
IOHandler.hs creates Graph using DataTypes.hs
    ↓
Main.hs calls algorithms in Processing.hs
    ↓
Processing.hs uses helper functions from Utils.hs
    ↓
Results go back through IOHandler.hs to show user
```

---

## 📊 Complete Feature List

### What This Program Can Do:

- **Find Shortest Path** - Between any two locations
- **Compare Algorithms** - See which method works best
- **Check Connectivity** - Test if locations are reachable
- **Show Statistics** - Graph size, degree, distances
- **Shortest Path Tree** - All routes from one location
- **Save Results** - Export findings to file
- **Beautiful Display** - Clear, formatted output
- **Error Handling** - Helpful messages for mistakes

### 🎯 4 Algorithms Included:

| Algorithm | Speed | Finds Optimal? | Best Use Case |
|-----------|-------|----------------|---------------|
| **BFS** | Fast | Yes (unweighted) | Fewest stops |
| **DFS** | Fast | No | Any path |
| **Dijkstra** | Medium | Yes | Shortest distance |
| **A*** | Fastest | Yes | Smart shortest |

---

## 🎮 Interactive Menu Guide

When you run the program:

```
+======================================================+
|   GRAPH TRAVERSAL & PATHFINDING SYSTEM              |
+======================================================+

Main Menu:
  1. Find path between two nodes        👈 Try this first!
  2. Compare all algorithms             👈 See differences
  3. Analyze graph connectivity         👈 Check reachability
  4. Display graph information          👈 View city stats
  5. Find shortest paths from a node    👈 All routes from one place
  6. Exit                               👈 Close program
```

**Pro Tips:**
- Start with option 1 to get familiar
- Try option 2 to see how algorithms differ
- Node IDs are single letters: A, B, C, D, E, F, G, H, I, J
- Type exactly as shown (case-sensitive)

---

## ❓ Troubleshooting

### Problem: "Module not found"

```powershell
# Solution: Check you're in the right folder
cd "e:\new projects\FP Group project"
Get-ChildItem  # Should see all .hs files
```

### Problem: "city_network.txt not found"

```powershell
# Solution: Verify file exists
Get-ChildItem city_network.txt  # Should show the file
```

### Problem: GHC not installed

```powershell
# Solution: Install Haskell Platform
# Download from: https://www.haskell.org/platform/
```

---

## 🎓 What You'll Learn

By studying this project, you'll understand:

### Functional Programming:
- Writing pure functions (no side effects)
- Using immutable data (can't change after creation)
- Recursion instead of loops
- Pattern matching for clean code
- Higher-order functions (functions using functions)

### Algorithms & Data Structures:
- Graph representation (adjacency lists)
- BFS and DFS traversal
- Dijkstra's shortest path
- A* heuristic search
- Priority queues

### Software Engineering:
- Modular code organization
- Separation of concerns (pure vs IO)
- Type-driven development
- Error handling
- User-friendly interfaces

---

## 🌟 Why Functional Programming is Awesome

### Safer Code
```
No mutable state -> No unexpected bugs
Type system -> Catch errors at compile time
Pure functions -> Easy to test
```

### Better Performance
```
Immutable data -> Safe parallelization
Lazy evaluation -> Compute only when needed
Pure functions -> Easy to optimize
```

### Easier Maintenance
```
No side effects -> Easy to understand
Pattern matching -> Clear logic flow
Type signatures -> Self-documenting
```

---
