{-|
Module      : Processing
Description : Graph traversal and pathfinding algorithms
Copyright   : (c) 2025
License     : MIT
Maintainer  : student@example.com

This module implements core graph traversal algorithms as pure functions.
All algorithms are referentially transparent and demonstrate functional programming
principles including immutability, recursion, and higher-order functions.

Algorithms implemented:
- Breadth-First Search (BFS) - Unweighted shortest path
- Depth-First Search (DFS) - Deep exploration
- Dijkstra's Algorithm - Weighted shortest path
- A* Algorithm - Heuristic-guided shortest path
-}

module Processing
    ( -- * Main Algorithm Interface
      findPath
    , findPathWithAlgorithm
    
    -- * Individual Algorithms
    , bfsPath
    , dfsPath
    , dijkstraPath
    , astarPath
    
    -- * Batch Processing
    , findAllPaths
    , compareAlgorithms
    
    -- * Graph Analysis
    , isConnected
    , findAllReachable
    , shortestPathTree
    ) where

import qualified Data.Map.Strict as Map
import qualified Data.Set as Set
import Data.Maybe (fromMaybe, isJust, mapMaybe)
import Data.List (minimumBy, foldl')
-- import Control.Parallel.Strategies (parMap, rdeepseq)
import DataTypes
import Utils

-- * Main Algorithm Interface

-- | Find a path using the default algorithm (Dijkstra)
findPath :: Graph -> NodeId -> NodeId -> PathResult
findPath = findPathWithAlgorithm Dijkstra

-- | Find a path using a specific algorithm
findPathWithAlgorithm :: AlgorithmType -> Graph -> NodeId -> NodeId -> PathResult
findPathWithAlgorithm algo graph start end
    | not (nodeExists graph start) = InvalidNode start
    | not (nodeExists graph end) = InvalidNode end
    | otherwise = case algo of
        BFS      -> bfsPath graph start end
        DFS      -> dfsPath graph start end
        Dijkstra -> dijkstraPath graph start end
        AStar    -> astarPath graph start end

-- * Breadth-First Search (BFS)

-- | BFS finds the shortest path in terms of number of edges (unweighted)
bfsPath :: Graph -> NodeId -> NodeId -> PathResult
bfsPath graph start end = bfsSearch [start] Set.empty Map.empty
  where
    bfsSearch :: [NodeId] -> Set.Set NodeId -> Map.Map NodeId NodeId -> PathResult
    bfsSearch [] _ _ = PathNotFound
    bfsSearch (current:rest) visited parents
        | current == end = 
            let pathNodes = reconstructPath parents start end
                cost = calculateCost graph pathNodes
            in PathFound (Path pathNodes cost)
        | Set.member current visited = bfsSearch rest visited parents
        | otherwise =
            let neighbors = map fst $ getNeighbors graph current
                unvisitedNeighbors = filter (`Set.notMember` visited) neighbors
                newQueue = rest ++ unvisitedNeighbors
                newParents = foldl' (\p n -> Map.insert n current p) parents unvisitedNeighbors
                newVisited = Set.insert current visited
            in bfsSearch newQueue newVisited newParents

-- * Depth-First Search (DFS)

-- | DFS explores as deeply as possible before backtracking
dfsPath :: Graph -> NodeId -> NodeId -> PathResult
dfsPath graph start end = dfsSearch [start] Set.empty Map.empty
  where
    dfsSearch :: [NodeId] -> Set.Set NodeId -> Map.Map NodeId NodeId -> PathResult
    dfsSearch [] _ _ = PathNotFound
    dfsSearch (current:rest) visited parents
        | current == end =
            let pathNodes = reconstructPath parents start end
                cost = calculateCost graph pathNodes
            in PathFound (Path pathNodes cost)
        | Set.member current visited = dfsSearch rest visited parents
        | otherwise =
            let neighbors = map fst $ getNeighbors graph current
                unvisitedNeighbors = filter (`Set.notMember` visited) neighbors
                newStack = unvisitedNeighbors ++ rest
                newParents = foldl' (\p n -> Map.insert n current p) parents unvisitedNeighbors
                newVisited = Set.insert current visited
            in dfsSearch newStack newVisited newParents

-- * Dijkstra's Algorithm

-- | Dijkstra's algorithm finds the shortest weighted path
dijkstraPath :: Graph -> NodeId -> NodeId -> PathResult
dijkstraPath graph start end =
    let initialPQ = insertPQ 0 start emptyPQ
        initialDist = Map.singleton start 0
    in dijkstraSearch initialPQ initialDist Map.empty Set.empty
  where
    dijkstraSearch :: PriorityQueue NodeId -> Map.Map NodeId Weight -> 
                      Map.Map NodeId NodeId -> Set.Set NodeId -> PathResult
    dijkstraSearch pq distances parents visited =
        case extractMinPQ pq of
            Nothing -> PathNotFound
            Just (current, restPQ)
                | current == end ->
                    let pathNodes = reconstructPath parents start end
                        cost = fromMaybe 0 $ Map.lookup end distances
                    in PathFound (Path pathNodes cost)
                | Set.member current visited ->
                    dijkstraSearch restPQ distances parents visited
                | otherwise ->
                    let currentDist = fromMaybe infinity $ Map.lookup current distances
                        neighbors = getNeighbors graph current
                        (newPQ, newDist, newParents) = 
                            foldl' (processNeighbor current currentDist) 
                                   (restPQ, distances, parents) 
                                   neighbors
                        newVisited = Set.insert current visited
                    in dijkstraSearch newPQ newDist newParents newVisited
    
    processNeighbor :: NodeId -> Weight -> 
                       (PriorityQueue NodeId, Map.Map NodeId Weight, Map.Map NodeId NodeId) ->
                       (NodeId, Weight) ->
                       (PriorityQueue NodeId, Map.Map NodeId Weight, Map.Map NodeId NodeId)
    processNeighbor current currentDist (pq, dist, parents) (neighbor, weight) =
        let newDist = currentDist + weight
            oldDist = fromMaybe infinity $ Map.lookup neighbor dist
        in if newDist < oldDist
           then (insertPQ newDist neighbor pq,
                 Map.insert neighbor newDist dist,
                 Map.insert neighbor current parents)
           else (pq, dist, parents)
    
    infinity :: Weight
    infinity = 1/0

-- * A* Algorithm

-- | A* uses a heuristic to guide the search more efficiently
astarPath :: Graph -> NodeId -> NodeId -> PathResult
astarPath graph start end =
    case (getNode graph start, getNode graph end) of
        (Just startNode, Just endNode) ->
            case (nodeCoord startNode, nodeCoord endNode) of
                (Just startCoord, Just endCoord) ->
                    let heuristic = euclideanDistance startCoord endCoord
                        initialPQ = insertPQ heuristic start emptyPQ
                        initialGScore = Map.singleton start 0
                    in astarSearch initialPQ initialGScore Map.empty Set.empty endCoord
                _ -> dijkstraPath graph start end  -- Fallback if no coordinates
        _ -> PathNotFound
  where
    astarSearch :: PriorityQueue NodeId -> Map.Map NodeId Weight ->
                   Map.Map NodeId NodeId -> Set.Set NodeId -> Coordinate -> PathResult
    astarSearch pq gScores parents visited endCoord =
        case extractMinPQ pq of
            Nothing -> PathNotFound
            Just (current, restPQ)
                | current == end ->
                    let pathNodes = reconstructPath parents start end
                        cost = fromMaybe 0 $ Map.lookup end gScores
                    in PathFound (Path pathNodes cost)
                | Set.member current visited ->
                    astarSearch restPQ gScores parents visited endCoord
                | otherwise ->
                    let currentG = fromMaybe infinity $ Map.lookup current gScores
                        neighbors = getNeighbors graph current
                        (newPQ, newGScores, newParents) =
                            foldl' (processNeighborAStar current currentG endCoord)
                                   (restPQ, gScores, parents)
                                   neighbors
                        newVisited = Set.insert current visited
                    in astarSearch newPQ newGScores newParents newVisited endCoord
    
    processNeighborAStar :: NodeId -> Weight -> Coordinate ->
                           (PriorityQueue NodeId, Map.Map NodeId Weight, Map.Map NodeId NodeId) ->
                           (NodeId, Weight) ->
                           (PriorityQueue NodeId, Map.Map NodeId Weight, Map.Map NodeId NodeId)
    processNeighborAStar current currentG endCoord (pq, gScores, parents) (neighbor, weight) =
        let tentativeG = currentG + weight
            oldG = fromMaybe infinity $ Map.lookup neighbor gScores
        in if tentativeG < oldG
           then let h = case getNode graph neighbor of
                          Just node -> case nodeCoord node of
                              Just coord -> euclideanDistance coord endCoord
                              Nothing -> 0
                          Nothing -> 0
                    fScore = tentativeG + h
                in (insertPQ fScore neighbor pq,
                    Map.insert neighbor tentativeG gScores,
                    Map.insert neighbor current parents)
           else (pq, gScores, parents)
    
    infinity :: Weight
    infinity = 1/0

-- * Batch Processing (demonstrates parallel processing capabilities)

-- | Find paths between multiple node pairs
-- Uses map (parallel processing commented out for compatibility)
findAllPaths :: AlgorithmType -> Graph -> [(NodeId, NodeId)] -> [PathResult]
findAllPaths algo graph pairs = map (uncurry $ findPathWithAlgorithm algo graph) pairs

-- | Compare all algorithms for a specific path
-- Returns results for each algorithm
compareAlgorithms :: Graph -> NodeId -> NodeId -> [SearchResult]
compareAlgorithms graph start end =
    map (\algo -> createSearchResult algo (findPathWithAlgorithm algo graph start end)) allAlgorithms
  where
    allAlgorithms = [BFS, DFS, Dijkstra, AStar]
    
    createSearchResult :: AlgorithmType -> PathResult -> SearchResult
    createSearchResult algo result =
        SearchResult
            { resultAlgorithm = algo
            , resultPath = result
            , resultNodesVisited = 0  -- Would need to instrument algorithms to track this
            , resultMaxFrontier = 0   -- Would need to instrument algorithms to track this
            }

-- * Graph Analysis Functions

-- | Check if the graph is fully connected
isConnected :: Graph -> Bool
isConnected graph =
    case Map.keys (graphNodes graph) of
        [] -> True
        (start:_) -> 
            let reachable = findAllReachable graph start
            in Set.size reachable == graphSize graph

-- | Find all nodes reachable from a starting node
findAllReachable :: Graph -> NodeId -> Set.Set NodeId
findAllReachable graph start = bfsReachable [start] Set.empty
  where
    bfsReachable :: [NodeId] -> Set.Set NodeId -> Set.Set NodeId
    bfsReachable [] visited = visited
    bfsReachable (current:rest) visited
        | Set.member current visited = bfsReachable rest visited
        | otherwise =
            let neighbors = map fst $ getNeighbors graph current
                newQueue = rest ++ neighbors
                newVisited = Set.insert current visited
            in bfsReachable newQueue newVisited

-- | Build a shortest path tree from a source node using Dijkstra
shortestPathTree :: Graph -> NodeId -> Map.Map NodeId Weight
shortestPathTree graph start =
    let initialPQ = insertPQ 0 start emptyPQ
        initialDist = Map.singleton start 0
    in dijkstraSPT initialPQ initialDist Set.empty
  where
    dijkstraSPT :: PriorityQueue NodeId -> Map.Map NodeId Weight -> Set.Set NodeId -> Map.Map NodeId Weight
    dijkstraSPT pq distances visited =
        case extractMinPQ pq of
            Nothing -> distances
            Just (current, restPQ)
                | Set.member current visited -> dijkstraSPT restPQ distances visited
                | otherwise ->
                    let currentDist = fromMaybe infinity $ Map.lookup current distances
                        neighbors = getNeighbors graph current
                        (newPQ, newDist) = foldl' (updateDistance current currentDist) (restPQ, distances) neighbors
                        newVisited = Set.insert current visited
                    in dijkstraSPT newPQ newDist newVisited
    
    updateDistance :: NodeId -> Weight -> (PriorityQueue NodeId, Map.Map NodeId Weight) -> 
                      (NodeId, Weight) -> (PriorityQueue NodeId, Map.Map NodeId Weight)
    updateDistance current currentDist (pq, dist) (neighbor, weight) =
        let newDist = currentDist + weight
            oldDist = fromMaybe infinity $ Map.lookup neighbor dist
        in if newDist < oldDist
           then (insertPQ newDist neighbor pq, Map.insert neighbor newDist dist)
           else (pq, dist)
    
    infinity :: Weight
    infinity = 1/0

-- * Helper Functions

-- | Calculate the cost of a path by summing edge weights
calculateCost :: Graph -> [NodeId] -> Weight
calculateCost graph nodes = sum $ zipWith getWeight nodes (tail nodes)
  where
    getWeight :: NodeId -> NodeId -> Weight
    getWeight from to =
        case lookup to (getNeighbors graph from) of
            Just w -> w
            Nothing -> 0
