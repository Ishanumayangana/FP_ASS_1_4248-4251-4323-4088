
module Main where

import System.IO
import System.Exit (exitSuccess)
import Control.Monad (when, forever)
import qualified Data.Map.Strict as Map
import DataTypes
import Utils
import Processing
import IOHandler

-- | Main entry point
main :: IO ()
main = do
    -- Set UTF-8 encoding for Windows compatibility
    hSetEncoding stdout utf8
    hSetEncoding stdin utf8
    
    -- Display welcome banner
    displayWelcome
    
    -- Load the graph data
    graphResult <- loadGraphFromFile "city_network.txt"
    
    case graphResult of
        Left err -> do
            putStrLn $ "Error loading graph: " ++ err
            putStrLn "Please ensure 'city_network.txt' exists in the current directory."
            exitSuccess
        Right graph -> do
            putStrLn "\n[SUCCESS] Graph loaded successfully!"
            displayGraphInfo graph
            
            -- Enter main application loop
            mainLoop graph

-- | Display welcome banner
displayWelcome :: IO ()
displayWelcome = do
    putStrLn ""
    putStrLn "+============================================================+"
    putStrLn "|                                                            |"
    putStrLn "|    GRAPH TRAVERSAL & PATHFINDING SYSTEM                    |"
    putStrLn "|    Functional Programming Project                          |"
    putStrLn "|                                                            |"
    putStrLn "|    Demonstrating:                                          |"
    putStrLn "|    * Pure Functional Programming                           |"
    putStrLn "|    * Multiple Graph Traversal Algorithms                   |"
    putStrLn "|    * Parallel Processing Capabilities                      |"
    putStrLn "|    * Real-world City Navigation                            |"
    putStrLn "|                                                            |"
    putStrLn "+============================================================+"
    putStrLn ""

-- | Main application loop
mainLoop :: Graph -> IO ()
mainLoop graph = do
    displayMenu
    choice <- getLine
    
    case choice of
        "1" -> handleFindPath graph
        "2" -> handleCompareAlgorithms graph
        "3" -> handleAnalyzeConnectivity graph
        "4" -> handleDisplayGraphInfo graph
        "5" -> handleShortestPathTree graph
        "6" -> handleExit
        _   -> do
            putStrLn "\n[ERROR] Invalid choice. Please enter a number between 1 and 6."
            waitForEnter
            mainLoop graph

-- | Handle finding a path between two nodes
handleFindPath :: Graph -> IO ()
handleFindPath graph = do
    putStrLn "\n+------------------------------------------+"
    putStrLn "|     FIND PATH BETWEEN TWO NODES          |"
    putStrLn "+------------------------------------------+"
    
    -- Get algorithm choice
    algorithm <- getAlgorithmChoice
    
    -- Get source and destination nodes
    source <- getNodeChoice graph "Select source node:"
    destination <- getNodeChoice graph "Select destination node:"
    
    -- Find the path
    putStrLn "\nComputing path..."
    let result = findPathWithAlgorithm algorithm graph source destination
    
    -- Display result
    displayPathResult result
    
    -- Offer to save results
    case result of
        PathFound path -> do
            save <- getUserInput "\nSave this path to file? (y/n): "
            when (save `elem` ["y", "Y", "yes"]) $ do
                filename <- getUserInput "Enter filename (e.g., path_result.txt): "
                exportPath filename path
        _ -> return ()
    
    waitForEnter
    mainLoop graph

-- | Handle comparing all algorithms
handleCompareAlgorithms :: Graph -> IO ()
handleCompareAlgorithms graph = do
    putStrLn "\n+------------------------------------------+"
    putStrLn "|     COMPARE ALL ALGORITHMS               |"
    putStrLn "+------------------------------------------+"
    
    -- Get source and destination nodes
    source <- getNodeChoice graph "Select source node:"
    destination <- getNodeChoice graph "Select destination node:"
    
    -- Compare all algorithms
    putStrLn "\nRunning all algorithms..."
    let results = compareAlgorithms graph source destination
    
    -- Display comparison
    displayAllResults results
    
    -- Analyze results
    analyzeComparison results
    
    -- Offer to save results
    save <- getUserInput "\nSave comparison results to file? (y/n): "
    when (save `elem` ["y", "Y", "yes"]) $ do
        filename <- getUserInput "Enter filename (e.g., comparison.txt): "
        saveResultsToFile filename results
    
    waitForEnter
    mainLoop graph

-- | Analyze and display comparison insights
analyzeComparison :: [SearchResult] -> IO ()
analyzeComparison results = do
    putStrLn "\nAnalysis:"
    
    let successfulPaths = [(resultAlgorithm r, p) | 
                           r <- results, 
                           PathFound p <- [resultPath r]]
    
    if null successfulPaths
        then putStrLn "  No algorithms found a path."
        else do
            let costs = [(algo, pathCost p) | (algo, p) <- successfulPaths]
            let minCost = minimum [c | (_, c) <- costs]
            let optimalAlgos = [algo | (algo, cost) <- costs, cost == minCost]
            
            putStrLn $ "  - Best cost: " ++ show minCost
            putStrLn $ "  - Optimal algorithm(s): " ++ show optimalAlgos
            
            -- Compare path lengths
            let lengths = [(algo, pathLength p) | (algo, p) <- successfulPaths]
            let shortestLength = minimum [l | (_, l) <- lengths]
            putStrLn $ "  - Shortest path (hops): " ++ show shortestLength

-- | Handle analyzing graph connectivity
handleAnalyzeConnectivity :: Graph -> IO ()
handleAnalyzeConnectivity graph = do
    putStrLn "\n+------------------------------------------+"
    putStrLn "|     GRAPH CONNECTIVITY ANALYSIS          |"
    putStrLn "+------------------------------------------+"
    
    -- Check if graph is connected
    let connected = isConnected graph
    
    putStrLn $ "\nGraph connectivity: " ++ 
               (if connected then "CONNECTED" else "DISCONNECTED")
    
    -- Analyze reachability from a specific node
    source <- getNodeChoice graph "\nSelect a node to analyze reachability:"
    
    let reachable = findAllReachable graph source
    let totalNodes = graphSize graph
    let reachableCount = length reachable
    
    putStrLn $ "\nFrom node '" ++ source ++ "':"
    putStrLn $ "  - Reachable nodes: " ++ show reachableCount ++ "/" ++ show totalNodes
    putStrLn $ "  - Coverage: " ++ show (percentage reachableCount totalNodes) ++ "%"
    
    waitForEnter
    mainLoop graph
  where
    percentage :: Int -> Int -> Double
    percentage part total = 
        fromIntegral (part * 100) / fromIntegral total

-- | Handle displaying graph information
handleDisplayGraphInfo :: Graph -> IO ()
handleDisplayGraphInfo graph = do
    displayGraphInfo graph
    
    -- Additional statistics
    putStrLn "\nAdditional Statistics:"
    
    let avgDegree = calculateAverageDegree graph
    putStrLn $ "  - Average node degree: " ++ show avgDegree
    
    let (maxDegree, maxNode) = findMaxDegreeNode graph
    putStrLn $ "  - Max degree: " ++ show maxDegree ++ " (node: " ++ maxNode ++ ")"
    
    waitForEnter
    mainLoop graph
  where
    calculateAverageDegree :: Graph -> Double
    calculateAverageDegree g =
        let degrees = [length edges | edges <- Map.elems (graphEdges g)]
            totalDegree = sum degrees
            nodeCount = graphSize g
        in if nodeCount > 0
           then fromIntegral totalDegree / fromIntegral nodeCount
           else 0
    
    findMaxDegreeNode :: Graph -> (Int, NodeId)
    findMaxDegreeNode g =
        let degrees = [(length edges, nid) | (nid, edges) <- Map.toList (graphEdges g)]
        in if null degrees
           then (0, "")
           else foldr1 (\a@(d1, _) b@(d2, _) -> if d1 > d2 then a else b) degrees

-- | Handle finding shortest path tree
handleShortestPathTree :: Graph -> IO ()
handleShortestPathTree graph = do
    putStrLn "\n+------------------------------------------+"
    putStrLn "|     SHORTEST PATH TREE                   |"
    putStrLn "+------------------------------------------+"
    
    source <- getNodeChoice graph "Select source node for shortest path tree:"
    
    putStrLn "\nComputing shortest paths from source..."
    let spt = shortestPathTree graph source
    
    putStrLn $ "\nShortest Path Tree from '" ++ source ++ "':"
    putStrLn "\n  Node      | Distance"
    putStrLn $ "  " ++ replicate 30 '-'
    
    mapM_ (\(node, dist) -> 
        putStrLn $ "  " ++ padRight 10 node ++ "| " ++ show dist) 
        (Map.toList spt)
    
    -- Find furthest node
    if not (Map.null spt)
        then do
            let (furthestNode, maxDist) = Map.foldrWithKey 
                    (\k v acc@(_, maxV) -> if v > maxV then (k, v) else acc) 
                    ("", 0) 
                    spt
            putStrLn $ "\n  Furthest reachable node: " ++ furthestNode 
                      ++ " (distance: " ++ show maxDist ++ ")"
        else putStrLn "\n  No reachable nodes from source."
    
    waitForEnter
    mainLoop graph
  where
    padRight :: Int -> String -> String
    padRight n s = take n (s ++ repeat ' ')

-- | Handle exit
handleExit :: IO ()
handleExit = do
    putStrLn "\n+============================================================+"
    putStrLn "|                                                            |"
    putStrLn "|    Thank you for using Graph Traversal System!            |"
    putStrLn "|                                                            |"
    putStrLn "|    This project demonstrates:                              |"
    putStrLn "|    * Pure functional programming principles               |"
    putStrLn "|    * Immutable data structures                             |"
    putStrLn "|    * Type safety and correctness                           |"
    putStrLn "|    * Modular, testable code design                         |"
    putStrLn "|                                                            |"
    putStrLn "+============================================================+\n"
    exitSuccess

-- * Additional Utility Functions

-- | Demonstrate batch processing with parallel computation
demonstrateParallelProcessing :: Graph -> IO ()
demonstrateParallelProcessing graph = do
    putStrLn "\nDemonstrating Parallel Processing..."
    
    let nodePairs = [("A", "D"), ("B", "E"), ("C", "F"), ("A", "F")]
    let results = findAllPaths Dijkstra graph nodePairs
    
    putStrLn "\nBatch processing results:"
    mapM_ displayPathResult results
