
module DataTypes
    ( -- * Graph Data Types
      NodeId
    , Node(..)
    , Edge(..)
    , Weight
    , Graph(..)
    
    -- * Path and Algorithm Result Types
    , Path(..)
    , PathResult(..)
    , AlgorithmType(..)
    , SearchResult(..)
    
    -- * Coordinate Types for A* Heuristic
    , Coordinate(..)
    , Distance
    
    -- * Smart Constructors
    , mkNode
    , mkEdge
    , mkGraph
    , mkPath
    ) where

import qualified Data.Map.Strict as Map
import qualified Data.Set as Set

-- | Type alias for node identifiers
type NodeId = String

-- | Type alias for edge weights (distances, costs, etc.)
type Weight = Double

-- | Type alias for distances in coordinate space
type Distance = Double

-- | Represents a node in the graph with optional coordinates for spatial algorithms
data Node = Node
    { nodeId   :: NodeId           -- ^ Unique identifier for the node
    , nodeName :: String           -- ^ Human-readable name
    , nodeCoord :: Maybe Coordinate -- ^ Optional coordinates for A* heuristic
    } deriving (Show, Eq, Ord)

-- | Represents 2D coordinates for spatial heuristics
data Coordinate = Coordinate
    { coordX :: Double  -- ^ X coordinate (e.g., longitude)
    , coordY :: Double  -- ^ Y coordinate (e.g., latitude)
    } deriving (Show, Eq, Ord)

-- | Represents a directed edge in the graph
data Edge = Edge
    { edgeFrom   :: NodeId  -- ^ Source node
    , edgeTo     :: NodeId  -- ^ Destination node
    , edgeWeight :: Weight  -- ^ Weight/cost of traversing this edge
    } deriving (Show, Eq, Ord)

-- | Represents a weighted directed graph
-- Uses adjacency list representation for efficient traversal
data Graph = Graph
    { graphNodes :: Map.Map NodeId Node           -- ^ Map of all nodes
    , graphEdges :: Map.Map NodeId [(NodeId, Weight)]  -- ^ Adjacency list
    } deriving (Show, Eq)

-- | Represents a path through the graph
data Path = Path
    { pathNodes :: [NodeId]  -- ^ Sequence of nodes in the path
    , pathCost  :: Weight    -- ^ Total cost/distance of the path
    } deriving (Show, Eq)

-- | Result of a pathfinding operation
data PathResult
    = PathFound Path         -- ^ Successful path found
    | PathNotFound           -- ^ No path exists between nodes
    | InvalidNode NodeId     -- ^ One or more nodes don't exist in graph
    deriving (Show, Eq)

-- | Types of graph traversal algorithms supported
data AlgorithmType
    = BFS        -- ^ Breadth-First Search (unweighted shortest path)
    | DFS        -- ^ Depth-First Search (explores deeply)
    | Dijkstra   -- ^ Dijkstra's algorithm (weighted shortest path)
    | AStar      -- ^ A* algorithm (weighted with heuristic)
    deriving (Show, Eq, Ord, Enum, Bounded)

-- | Comprehensive search result with metadata
data SearchResult = SearchResult
    { resultAlgorithm    :: AlgorithmType  -- ^ Algorithm used
    , resultPath         :: PathResult     -- ^ Path result
    , resultNodesVisited :: Int            -- ^ Number of nodes explored
    , resultMaxFrontier  :: Int            -- ^ Maximum frontier size (memory usage)
    } deriving (Show, Eq)

-- * Smart Constructors

-- | Smart constructor for Node with validation
mkNode :: NodeId -> String -> Maybe Coordinate -> Node
mkNode nid name coord
    | null nid = error "Node ID cannot be empty"
    | null name = error "Node name cannot be empty"
    | otherwise = Node nid name coord

-- | Smart constructor for Edge with validation
mkEdge :: NodeId -> NodeId -> Weight -> Edge
mkEdge from to weight
    | null from || null to = error "Node IDs cannot be empty"
    | weight < 0 = error "Edge weight must be non-negative"
    | from == to = error "Self-loops not allowed"
    | otherwise = Edge from to weight

-- | Smart constructor for Graph
mkGraph :: [Node] -> [Edge] -> Graph
mkGraph nodes edges =
    let nodeMap = Map.fromList [(nodeId n, n) | n <- nodes]
        adjList = buildAdjacencyList edges
    in Graph nodeMap adjList
  where
    buildAdjacencyList :: [Edge] -> Map.Map NodeId [(NodeId, Weight)]
    buildAdjacencyList = foldr addEdge Map.empty
    
    addEdge :: Edge -> Map.Map NodeId [(NodeId, Weight)] -> Map.Map NodeId [(NodeId, Weight)]
    addEdge (Edge from to weight) =
        Map.insertWith (++) from [(to, weight)]

-- | Smart constructor for Path with validation
mkPath :: [NodeId] -> Weight -> Path
mkPath nodes cost
    | null nodes = error "Path cannot be empty"
    | cost < 0 = error "Path cost must be non-negative"
    | otherwise = Path nodes cost

-- * Instance Declarations for pretty printing

instance Ord Path where
    compare p1 p2 = compare (pathCost p1) (pathCost p2)
