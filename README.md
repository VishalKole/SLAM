# Parallel-Graph-Metric-Computation

The distance between two vertices is the length of the shortest path between those two vertices-that is, the number of edges in this shortest path. If there is no path between the two vertices, the distance between them is infinity.  
The **eccentricity** of a vertex is the largest distance between that vertex and any other vertex.  
The **radius** of a graph is the smallest eccentricity of any vertex in the graph.  
The **diameter** of a graph is the largest eccentricity of any vertex in the graph.  
A **central vertex** in a graph is a vertex whose eccentricity is equal to the graph's radius.  
A **peripheral vertex** in a graph is a vertex whose eccentricity is equal to the graph's diameter. 

For example, here is a graph with 10 vertices and 12 edges:  
![image removed](https://www.cs.rit.edu/~ark/fall2017/654/p2/g1.png)

The vertices' eccentricities are
```
Vertex        0  1  2  3  4  5  6  7  8  9
Eccentricity  2  4  3  4  3  3  3  4  3  3
```
The graph's radius is 2. The graph's diameter is 4. The graph's central vertex is 0. The graph's peripheral vertices are 1, 3, and 7.

The program's command line argument is a constructor expression for a class that implements interface GraphSpec. The program uses the Instance.newInstance() method in package edu.rit.util in the Parallel Java 2 Library to construct a graph spec object, like so:
```
GraphSpec gs = (GraphSpec) Instance.newInstance (args[0]);
```
The program then calls methods on the graph spec object to obtain the number of vertices, the number of edges, and the edges themselves in the graph to be analyzed. 

## Software Specification 
The program must be run by typing this command line:
```
java pj2 jar=<jarfile> workers=<K> GraphMetrics "<ctor>"
```
* `<jarfile>` is the name of a Java archive file containing all the Java class files in the program.  
* `<K>` is the number of worker tasks.  
* `<ctor>` is a constructor expression for a class that implements interface GraphSpec; the object created by this constructor expression specifies the graph to be analyzed.

#### If the command line does not have the required number of arguments, if any argument is erroneous, or if any other error occurs, the program prints an error message on the console and exits.