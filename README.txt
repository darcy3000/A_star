Implementing A Star:
We downloaded osm file for Delhi and converted it into a post-gre sql database using osm2pgsql module.
We are dynamically creating the graph using nodes from database by finding neighbours a node only if it has been pushed in priority queue.
We have haversine distance as the heuristic function.
We have used googlmaps api to find distance along roads between two nodes.
We have converted latitude and longitude from EPSG3857 to SRID 4326.
Then we have implemented A* search.
