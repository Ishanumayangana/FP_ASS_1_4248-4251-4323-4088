{-|
Module      : IOHandler
Description : Input/output operations and user interaction
Copyright   : (c) 2025
License     : MIT
Maintainer  : student@example.com

This module handles all IO operations, keeping the boundary between
pure functional code and side effects clear. It demonstrates how FP
manages IO through the IO monad while keeping core logic pure.
-}

module IOHandler
    ( -- * Graph Loading
      loadGraphFromFile
    , parseGraphData
    
    -- * User Interaction
    , getUserInput
    , displayMenu
    , getAlgorithmChoice
    , getNodeChoice
    
    -- * Result Display
    , displayPathResult
    , displaySearchResult
    , displayGraphInfo
    , displayAllResults
    
    -- * File Operations
    , saveResultsToFile
    , exportPath
    
    -- * Helper Functions
    , waitForEnter
    ) where

import System.IO
import Control.Monad (when, forM_)
import Data.List (intercalate)
import qualified Data.Map.Strict as Map
import Text.Read (readMaybe)
import DataTypes
import Utils
import Processing

-- * Graph Loading Functions

-- | Load a graph from a file
-- File format:
--   NODES
--   node_id,name,x,y
--   EDGES
--   from,to,weight
loadGraphFromFile :: FilePath -> IO (Either String Graph)
loadGraphFromFile filepath = do
    contents <- readFile filepath
    return $ parseGraphData contents

-- | Parse graph data from string content
parseGraphData :: String -> Either String Graph
parseGraphData content =
    let lns = lines content
        (nodeLines, edgeLines) = splitSections lns
    in do
        nodes <- mapM parseNodeLine nodeLines
        edges <- mapM parseEdgeLine edgeLines
        return $ mkGraph nodes edges
  where
    splitSections :: [String] -> ([String], [String])
    splitSections ls =
        let (beforeEdges, afterEdges) = break (== "EDGES") $ dropWhile (/= "NODES") ls
            nodeLines = filter (not . null) $ drop 1 $ takeWhile (/= "EDGES") beforeEdges
            edgeLines = filter (not . null) $ drop 1 afterEdges
        in (nodeLines, edgeLines)
    
    parseNodeLine :: String -> Either String Node
    parseNodeLine line =
        case splitOn ',' line of
            [nid, name, x, y] ->
                case (readMaybe x, readMaybe y) of
                    (Just xCoord, Just yCoord) ->
                        Right $ mkNode nid name (Just $ Coordinate xCoord yCoord)
                    _ -> Left $ "Invalid coordinates in node: " ++ line
            [nid, name] ->
                Right $ mkNode nid name Nothing
            _ -> Left $ "Invalid node format: " ++ line
    
    parseEdgeLine :: String -> Either String Edge
    parseEdgeLine line =
        case splitOn ',' line of
            [from, to, weight] ->
                case readMaybe weight of
                    Just w -> Right $ mkEdge from to w
                    Nothing -> Left $ "Invalid weight in edge: " ++ line
            _ -> Left $ "Invalid edge format: " ++ line
    
    splitOn :: Char -> String -> [String]
    splitOn _ "" = []
    splitOn c s =
        let (first, rest) = break (== c) s
        in first : case rest of
            [] -> []
            (_:rs) -> splitOn c rs

-- * User Interaction Functions

-- | Display the main menu
displayMenu :: IO ()
displayMenu = do
    putStrLn "\n+======================================================+"
    putStrLn "|     GRAPH TRAVERSAL & PATHFINDING SYSTEM            |"
    putStrLn "|     Functional Programming Project                   |"
    putStrLn "+======================================================+"
    putStrLn "\nMain Menu:"
    putStrLn "  1. Find path between two nodes"
    putStrLn "  2. Compare all algorithms"
    putStrLn "  3. Analyze graph connectivity"
    putStrLn "  4. Display graph information"
    putStrLn "  5. Find shortest paths from a node"
    putStrLn "  6. Exit"
    putStr "\nEnter your choice (1-6): "
    hFlush stdout

-- | Get user input with a prompt
getUserInput :: String -> IO String
getUserInput prompt = do
    putStr prompt
    hFlush stdout
    getLine

-- | Get algorithm choice from user
getAlgorithmChoice :: IO AlgorithmType
getAlgorithmChoice = do
    putStrLn "\nSelect Algorithm:"
    putStrLn "  1. BFS (Breadth-First Search)"
    putStrLn "  2. DFS (Depth-First Search)"
    putStrLn "  3. Dijkstra's Algorithm"
    putStrLn "  4. A* Algorithm"
    choice <- getUserInput "Enter choice (1-4): "
    case choice of
        "1" -> return BFS
        "2" -> return DFS
        "3" -> return Dijkstra
        "4" -> return AStar
        _ -> do
            putStrLn "Invalid choice. Using Dijkstra by default."
            return Dijkstra

-- | Get node ID from user with validation
getNodeChoice :: Graph -> String -> IO NodeId
getNodeChoice graph prompt = do
    putStrLn $ "\n" ++ prompt
    displayNodeList graph
    nodeId <- getUserInput "Enter node ID: "
    if nodeExists graph nodeId
        then return nodeId
        else do
            putStrLn $ "Error: Node '" ++ nodeId ++ "' does not exist."
            getNodeChoice graph prompt

-- | Display list of available nodes
displayNodeList :: Graph -> IO ()
displayNodeList graph = do
    putStrLn "Available nodes:"
    let nodes = Map.elems (graphNodes graph)
    forM_ nodes $ \node ->
        putStrLn $ "  - " ++ nodeId node ++ ": " ++ nodeName node

-- * Result Display Functions

-- | Display a path result with formatting
displayPathResult :: PathResult -> IO ()
displayPathResult result = do
    putStrLn "\n+------------------------------------------+"
    putStrLn "|         PATH RESULT                      |"
    putStrLn "+------------------------------------------+"
    case result of
        PathFound path -> do
            putStrLn "[SUCCESS] Path found!"
            putStrLn $ "  Route: " ++ intercalate " -> " (pathNodes path)
            putStrLn $ "  Total cost: " ++ show (pathCost path)
            putStrLn $ "  Number of hops: " ++ show (pathLength path)
        PathNotFound ->
            putStrLn "[FAILED] No path exists between the specified nodes."
        InvalidNode nid ->
            putStrLn $ "[ERROR] Invalid node: " ++ nid

-- | Display a search result with algorithm metadata
displaySearchResult :: SearchResult -> IO ()
displaySearchResult result = do
    putStrLn $ "\nAlgorithm: " ++ show (resultAlgorithm result)
    displayPathResult (resultPath result)

-- | Display all search results in a comparison
displayAllResults :: [SearchResult] -> IO ()
displayAllResults results = do
    putStrLn "\n+======================================================+"
    putStrLn "|         ALGORITHM COMPARISON                         |"
    putStrLn "+======================================================+"
    forM_ results displaySearchResult
    putStrLn $ "\n" ++ replicate 54 '-'

-- | Display graph information
displayGraphInfo :: Graph -> IO ()
displayGraphInfo graph = do
    putStrLn "\n+======================================================+"
    putStrLn "|         GRAPH INFORMATION                            |"
    putStrLn "+======================================================+"
    putStrLn $ "  Number of nodes: " ++ show (graphSize graph)
    putStrLn $ "  Number of edges: " ++ show (countEdges graph)
    putStrLn $ "  Is connected: " ++ show (isConnected graph)
    putStrLn "\nNodes in graph:"
    displayNodeList graph
  where
    countEdges :: Graph -> Int
    countEdges g = sum [length edges | edges <- Map.elems (graphEdges g)]

-- | Display shortest path tree from a source
displayShortestPathTree :: Graph -> NodeId -> IO ()
displayShortestPathTree graph source = do
    let tree = shortestPathTree graph source
    putStrLn $ "\nShortest distances from " ++ source ++ ":"
    forM_ (Map.toList tree) $ \(node, distance) ->
        putStrLn $ "  " ++ node ++ ": " ++ show distance

-- * File Operations

-- | Save pathfinding results to a file
saveResultsToFile :: FilePath -> [SearchResult] -> IO ()
saveResultsToFile filepath results = do
    let content = unlines $ map formatSearchResult results
    writeFile filepath content
    putStrLn $ "\n[SUCCESS] Results saved to: " ++ filepath
  where
    formatSearchResult :: SearchResult -> String
    formatSearchResult result =
        let algo = show (resultAlgorithm result)
            pathInfo = case resultPath result of
                PathFound path ->
                    "Path: " ++ intercalate " -> " (pathNodes path) ++
                    ", Cost: " ++ show (pathCost path)
                PathNotFound -> "No path found"
                InvalidNode nid -> "Invalid node: " ++ nid
        in algo ++ " | " ++ pathInfo

-- | Export a single path to a file
exportPath :: FilePath -> Path -> IO ()
exportPath filepath path = do
    let content = unlines
            [ "Path Export"
            , "==========="
            , "Nodes: " ++ intercalate " -> " (pathNodes path)
            , "Cost: " ++ show (pathCost path)
            , "Length: " ++ show (pathLength path)
            ]
    writeFile filepath content
    putStrLn $ "\n[SUCCESS] Path exported to: " ++ filepath

-- * Helper Functions

-- | Read an integer from user input
readInt :: String -> IO (Maybe Int)
readInt prompt = do
    input <- getUserInput prompt
    return $ readMaybe input

-- | Confirm action with user
confirmAction :: String -> IO Bool
confirmAction prompt = do
    response <- getUserInput $ prompt ++ " (y/n): "
    return $ response `elem` ["y", "Y", "yes", "Yes", "YES"]

-- | Display error message with formatting
displayError :: String -> IO ()
displayError msg = do
    putStrLn $ "\n[ERROR] " ++ msg

-- | Display success message with formatting
displaySuccess :: String -> IO ()
displaySuccess msg = do
    putStrLn $ "\n[SUCCESS] " ++ msg

-- | Wait for user to press Enter
waitForEnter :: IO ()
waitForEnter = do
    putStr "\nPress Enter to continue..."
    hFlush stdout
    _ <- getLine
    return ()

-- | Clear screen (works on both Windows and Unix)
clearScreen :: IO ()
clearScreen = putStr "\ESC[2J\ESC[H"

-- | Display a separator line
displaySeparator :: IO ()
displaySeparator = putStrLn $ replicate 54 '─'
