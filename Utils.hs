{-|
Module      : Utils
Description : Utility functions for graph operations and algorithms
Copyright   : (c) 2025
License     : MIT
Maintainer  : student@example.com

This module provides pure utility functions used throughout the application.
All functions are referentially transparent and side-effect free.
-}

module Utils
    ( -- * Graph Query Functions
      getNode
    , getNeighbors
    , nodeExists
    , edgeExists
    , graphSize
    
    -- * Priority Queue (Min-Heap)
    , PriorityQueue
    , emptyPQ
    , insertPQ
    , extractMinPQ
    , isEmptyPQ
    
    -- * Distance and Heuristic Functions
    , euclideanDistance
    , manhattanDistance
    , reconstructPath
    
    -- * Path Utilities
    , pathLength
    , isValidPath
    , formatPath
    
    -- * General Utilities
    , safeHead
    , safeLast
    , unique
    ) where

import qualified Data.Map.Strict as Map
import qualified Data.Set as Set
import Data.List (foldl', nub)
import Data.Maybe (isJust, fromMaybe)
import DataTypes

-- * Graph Query Functions

-- | Safely retrieve a node from the graph
getNode :: Graph -> NodeId -> Maybe Node
getNode graph nid = Map.lookup nid (graphNodes graph)

-- | Get all neighbors of a node with their edge weights
getNeighbors :: Graph -> NodeId -> [(NodeId, Weight)]
getNeighbors graph nid = fromMaybe [] $ Map.lookup nid (graphEdges graph)

-- | Check if a node exists in the graph
nodeExists :: Graph -> NodeId -> Bool
nodeExists graph nid = isJust $ getNode graph nid

-- | Check if an edge exists between two nodes
edgeExists :: Graph -> NodeId -> NodeId -> Bool
edgeExists graph from to =
    case Map.lookup from (graphEdges graph) of
        Nothing -> False
        Just neighbors -> to `elem` map fst neighbors

-- | Get the number of nodes in the graph
graphSize :: Graph -> Int
graphSize = Map.size . graphNodes

-- * Priority Queue Implementation
-- Min-heap for efficient priority-based algorithms like Dijkstra and A*

-- | Priority queue implemented as a list of (priority, value) pairs
-- More efficient implementations would use actual heap structures,
-- but this demonstrates pure functional approach clearly
type PriorityQueue a = [(Double, a)]

-- | Create an empty priority queue
emptyPQ :: PriorityQueue a
emptyPQ = []

-- | Insert an element with priority into the queue
-- Maintains sorted order for efficient extraction
insertPQ :: Double -> a -> PriorityQueue a -> PriorityQueue a
insertPQ priority item queue = insertSorted (priority, item) queue
  where
    insertSorted :: (Double, a) -> [(Double, a)] -> [(Double, a)]
    insertSorted x [] = [x]
    insertSorted x@(p, _) (y@(q, _):ys)
        | p <= q    = x : y : ys
        | otherwise = y : insertSorted x ys

-- | Extract the minimum priority element from the queue
-- Returns Nothing if queue is empty
extractMinPQ :: PriorityQueue a -> Maybe (a, PriorityQueue a)
extractMinPQ [] = Nothing
extractMinPQ ((_, item):rest) = Just (item, rest)

-- | Check if priority queue is empty
isEmptyPQ :: PriorityQueue a -> Bool
isEmptyPQ = null

-- * Distance and Heuristic Functions

-- | Calculate Euclidean distance between two coordinates
-- Used as heuristic for A* algorithm
euclideanDistance :: Coordinate -> Coordinate -> Distance
euclideanDistance (Coordinate x1 y1) (Coordinate x2 y2) =
    sqrt ((x2 - x1)^2 + (y2 - y1)^2)

-- | Calculate Manhattan distance between two coordinates
-- Alternative heuristic for grid-based graphs
manhattanDistance :: Coordinate -> Coordinate -> Distance
manhattanDistance (Coordinate x1 y1) (Coordinate x2 y2) =
    abs (x2 - x1) + abs (y2 - y1)

-- | Reconstruct path from a parent map
-- Commonly used in BFS, Dijkstra, and A* algorithms
reconstructPath :: Map.Map NodeId NodeId -> NodeId -> NodeId -> [NodeId]
reconstructPath parentMap start end = reverse $ buildPath end
  where
    buildPath :: NodeId -> [NodeId]
    buildPath current
        | current == start = [current]
        | otherwise = case Map.lookup current parentMap of
            Nothing -> [current]  -- Shouldn't happen with valid input
            Just parent -> current : buildPath parent

-- * Path Utilities

-- | Calculate the length (number of edges) of a path
pathLength :: Path -> Int
pathLength path = max 0 (length (pathNodes path) - 1)

-- | Verify that a path is valid in the given graph
-- Checks that all nodes exist and edges connect properly
isValidPath :: Graph -> Path -> Bool
isValidPath graph path = all nodeExistsCheck nodes && all edgeExistsCheck (zip nodes (tail nodes))
  where
    nodes = pathNodes path
    nodeExistsCheck n = nodeExists graph n
    edgeExistsCheck (from, to) = edgeExists graph from to

-- | Format a path as a readable string
formatPath :: Path -> String
formatPath path = 
    let nodeList = pathNodes path
        pathStr = concat $ intersperse " -> " nodeList
    in pathStr ++ " (Cost: " ++ show (pathCost path) ++ ")"
  where
    intersperse :: a -> [a] -> [a]
    intersperse _ [] = []
    intersperse _ [x] = [x]
    intersperse sep (x:xs) = x : sep : intersperse sep xs

-- * General Utilities

-- | Safe head function that returns Maybe
safeHead :: [a] -> Maybe a
safeHead [] = Nothing
safeHead (x:_) = Just x

-- | Safe last function that returns Maybe
safeLast :: [a] -> Maybe a
safeLast [] = Nothing
safeLast [x] = Just x
safeLast (_:xs) = safeLast xs

-- | Remove duplicates from list while preserving order
unique :: Eq a => [a] -> [a]
unique = nub

-- * Helper Functions for Algorithm Implementation

-- | Calculate total cost of a path given a list of nodes and edge weights
calculatePathCost :: Graph -> [NodeId] -> Weight
calculatePathCost graph nodes = sum $ zipWith (getEdgeWeight graph) nodes (tail nodes)
  where
    getEdgeWeight :: Graph -> NodeId -> NodeId -> Weight
    getEdgeWeight g from to =
        let neighbors = getNeighbors g from
        in case lookup to neighbors of
            Just weight -> weight
            Nothing -> 0  -- Should not happen in valid path

-- | Find the shortest path cost between two nodes in a list of edges
findEdgeWeight :: [(NodeId, Weight)] -> NodeId -> Maybe Weight
findEdgeWeight edges target = lookup target edges

-- | Merge two maps, preferring values from the second map on conflict
mergeMaps :: Ord k => Map.Map k v -> Map.Map k v -> Map.Map k v
mergeMaps = Map.union

-- | Check if all elements in a list satisfy a predicate
allSatisfy :: (a -> Bool) -> [a] -> Bool
allSatisfy = all

-- | Filter and map in one pass (more efficient than separate operations)
filterMap :: (a -> Maybe b) -> [a] -> [b]
filterMap f = foldr (\x acc -> case f x of
                                  Just y -> y : acc
                                  Nothing -> acc) []
