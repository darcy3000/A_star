import psycopg2
import heapq
import googlemaps
import re
import math as Math
from haversine import haversine
import overpy
import os
import sys

gmaps = googlemaps.Client(key='AIzaSyByr2-ot4y7xPZYhUjiccR4EtYa_UX51IU')
connection = psycopg2.connect(database="abhijeet",user='postgres',host="localhost")
cursor = connection.cursor()

priority_queue=[]#First element will be fn + gn and second element will be node
visited = set()#Checks if the given nodes is visited
distance = {}#Gets the distance from source
path ={}#decide which is the next node
list_of_points =[]
#Getting all the points from database
cursor.execute('''SELECT osm_id,lat,lon from planet_osm_point
INNER JOIN planet_osm_nodes on planet_osm_nodes.id = planet_osm_point.osm_id;''')
list_of_point = cursor.fetchone()
while(list_of_point is not None):
    list_of_points.append(list_of_point)
    list_of_point = cursor.fetchone()

final_points =[]

"""
#Get all the roads associated with the concerned node
cursor.execute('''SELECT id from planet_osm_roads INNER JOIN
(SELECT * from 
(SELECT id,unnest(nodes) as A from planet_osm_ways) as navneet where 
(A in (SELECT id from planet_osm_nodes where tags::text like '%Sarojini%'))) as abhijeet on abhijeet.id=planet_osm_roads.osm_id;''')

road_id = cursor.fetchone()
road_ids =[]
while(road_id is not None):
    print(road_id[0])
    road_ids.append(road_id[0])
    road_id = cursor.fetchone()
"""

def get_lat_long(longitude,latitude):
    lon = longitude * 180 / 20037508.34
    lat = Math.atan(Math.exp(latitude * Math.pi / 20037508.34)) * 360 / Math.pi - 90;
    return (lat, lon)


def get_road_on_node(node):#Return Road osm_id and returns ,along with the number
    cursor.execute('''
    SELECT planet_osm_ways.id from planet_osm_ways INNER JOIN
    (SELECT id from planet_osm_roads INNER JOIN
    (SELECT id from 
    (SELECT id,unnest(nodes) as A from planet_osm_ways) as navneet where 
    (A =%s  )) as abhijeet on abhijeet.id=planet_osm_roads.osm_id) 
    as road_ids on road_ids.id=planet_osm_ways.id;''',[node])
    roads_ids =[]
    roads_id = cursor.fetchone()
    while(roads_id is not None):
        roads_ids.append(roads_id)
        roads_id = cursor.fetchone()
    return roads_ids


#Get all the roads and nodes on that particular node
def get_road_with_associated_nodes(node):
    cursor.execute('''
    SELECT planet_osm_ways.id,unnest(planet_osm_ways.nodes) from planet_osm_ways INNER JOIN
    (SELECT id from planet_osm_roads INNER JOIN
    (SELECT * from 
    (SELECT id,unnest(nodes) as A from planet_osm_ways) as navneet where 
    (A = %s)) as abhijeet on abhijeet.id=planet_osm_roads.osm_id) 
    as road_ids on road_ids.id=planet_osm_ways.id;''',[node])

    road_node = cursor.fetchone()
    road_nodes = []#Contains pair where 1st element is road and 2nd element is node
    while(road_node is not None):
        road_nodes.append([road_node[0],road_node[1]])
        road_node = cursor.fetchone()
    return road_nodes

#Finding the nodes corresponding to source
def get_nodes_from_source(source):
    source = '%'+source+'%'
    cursor.execute('''SELECT id FROM planet_osm_nodes WHERE tags::text LIKE %s; ''',[source])
    source_nodes=[]
    source_node =cursor.fetchone()
    while(source_node is not None):
        source_nodes.append(source_node)
        source_node = cursor.fetchone()
    return source_nodes

def get_nodes_from_destination(destiantion):
    #Finding the nodes corresponding to destination(given as string)
    destiantion ='%'+destiantion+'%'
    cursor.execute('''SELECT id FROM planet_osm_nodes WHERE tags::text LIKE %s; ''',[destiantion])
    des_nodes=[]
    des_node = cursor.fetchone()
    while(des_node is not None):
        des_nodes.append(des_node)
        des_node = cursor.fetchone()
    return des_nodes


#Finding the points of source(given as node)

def get_geometry_from_nodes(node):#Returns Geometry of the given node
    cursor.execute('''SELECT ST_AsText(way) from planet_osm_point where osm_id=%s''',[node])
    point_geometry = cursor.fetchone()
    return point_geometry

def get_geometry_from_roads(road):
    cursor.execute('''SELECT ST_AsText(way) from planet_osm_roads where osm_id = %s;   ''',[road])
    road_geometry = cursor.fetchone()
    return  road_geometry

#Calculating the distance between 2 nodes along the road
def get_distance_along_road(road,point1,point2):
    try:
        point1_geom = get_geometry_from_nodes(point1)
        point2_geom = get_geometry_from_nodes(point2)
        road_geom = get_geometry_from_roads(road)
        if((point1_geom is None) or (point2_geom is None) or (road_geom is None)):
            return None
        road_geom = str(road_geom)[2:-2]
        point2_geom = str(point2_geom)[2:-2]
        point1_geom = str(point1_geom)[2:-2]
        cursor.execute('''SELECT
                    ST_Length(ST_LineSubstring(
                    line,
                    least(ST_LineLocatePoint(line, pta), ST_LineLocatePoint(line, ptb)),
                    greatest(ST_LineLocatePoint(line, pta), ST_LineLocatePoint(line, ptb)))::geography)
                    FROM (
                    SELECT
                    'SRID=4326;''' + road_geom + ''' ::geometry line,
                    'SRID=4326;''' + point1_geom + '''  ::geometry pta,
                    'SRID=4326;''' + point2_geom + ''' ::geometry ptb
                    ) data;''')
        dis = cursor.fetchone()
        return dis
    except Exception as e:
        print(e)
        print("Query Failed")



#Finding the points of source(given as string)
def get_geometry_from_source(source):
    source = '%'+source+'%'
    cursor.execute('''SELECT ST_AsText(way) from planet_osm_point where osm_id in(
    SELECT id FROM planet_osm_nodes WHERE tags::text LIKE %s );''',[source])
    source_points=[]
    source_point = cursor.fetchone()
    while(source_point is not None):
        source_points.append(source_point)
        source_point = cursor.fetchone()
    return source_points

def get_geometry_from_destination(destination):
    #Finding the points corresponding to destiantion
    destination = '%'+destination+'%'
    cursor.execute('''SELECT ST_AsText(way) from planet_osm_point where osm_id in(
    SELECT id FROM planet_osm_nodes WHERE tags::text LIKE %s);''',[destination])

    des_points = []
    des_point = cursor.fetchone()
    while(des_point is not None):
        des_points.append(des_point)
        des_point = cursor.fetchone()
    return des_points

def get_heuristic_value(node1,node2):
    source = get_latitutde_longitude(node1)
    des = get_latitutde_longitude(node2)
    x = get_lat_long(source[1],source[0])
    y = get_lat_long(des[1],des[0])
    return haversine(x,y)

def get_perpendicular_distance_between_road_node(road,node):
    node_geom = get_geometry_from_nodes(node)
    road_geom = get_geometry_from_roads(road)
    try:
        cursor.execute('''SELECT ST_Distance(%s,%s);''',[road_geom,node_geom])
        return cursor.fetchone()
    except:
        connection.rollback()
        print("Query Failed")

def get_names_from_node(node):
    cursor.execute('''SELECT name from planet_osm_point where osm_id = %s''',[node])
    return cursor.fetchone()

def get_latitutde_longitude(node):
    cursor.execute('''SELECT lat,lon from planet_osm_nodes where id = %s''',[node])
    ans = cursor.fetchone()
    val = (ans[0]/100,ans[1]/100)
    return val

def get_travel_distance(node1,node2):
    source = get_latitutde_longitude(node1)
    des = get_latitutde_longitude(node2)
    x = get_lat_long(source[1],source[0])
    y = get_lat_long(des[1],des[0])
    s = gmaps.distance_matrix(x,y)
    val =s['rows'][0]['elements'][0]['distance']['text']
    if('k' in val):
        val = val[:-3]
        return float(val)
    else:
        val=val[:-2]
        return (float(val)/1000.0)


def get_edges(node):
    api = overpy.Overpass()
    x = get_latitutde_longitude(node)
    y = get_lat_long(x[1],x[0])
    inner = api.query("node("+str(y[0])+","+str(y[1])+","+str(y[0]+0.05)+","+str(y[1]+0.05)+")['highway'];out;")
    #outer = api.query("node("+str(y[0])+","+str(y[1])+","+str(y[0]+0.010001)+","+str(y[1]+0.010001)+")['highway'];out;")

    node = inner.nodes
    #y = set(outer.nodes)
    #node = y.difference(x)
    val =[]
    for x in node:
        val.append(x.id)
    return val

def get_nodes(node):
    try:
        x = get_latitutde_longitude(node)
        y = get_lat_long(x[1],x[0])
        radius_nodes =[]
        final_list=[]
        for i in final_points:
            cursor.execute('''SELECT ST_Distance(
            ST_GeomFromText('POINT(%s %s)'),
            ST_GeomFromText('POINT(%s %s)')
            )''',[i[1][1],i[1][0],y[1],y[0]])
            wt = cursor.fetchone()[0]
            heapq.heappush(radius_nodes,([wt,i[0]]))

        i = 0
        while(i<6):

            z = heapq.heappop(radius_nodes)
            if(z[1] not in visited):
                final_list.append(z[1])
                i+=1

        return final_list


    except Exception as e:
        connection.rollback()
        print(e)
        print("Query Failed")


def get_graph(node):
    cursor.execute('''SELECT id from planet_osm_nodes p2 
    where exists(SELECT true from planet_osm_nodes  p1 where p1.id='''+str(node)+''' and 
    ((p2.lat<p1.lat+5000 and p2.lat>p1.lat-5000)and(p2.lon<p1.lon+5000 and p2.lon>p1.lon-5000)))   
    LIMIT 8;''')
    graph_node =cursor.fetchone()
    graph_nodes =[]
    while(graph_node is not None):
        graph_nodes.append(graph_node[0])
        graph_node = cursor.fetchone()
    return graph_nodes


def A_star(source_name,des_name):
    global priority_queue   # First element will be fn + gn and second element will be node
    global visited   # Checks if the given nodes is visited
    global distance   # Gets the distance from source
    global path   # decide which is the next node

    routes =[]
    flag = 0
    source_nodes = get_nodes_from_source(source_name)
    des_nodes = get_nodes_from_destination(des_name)
    des_points = []#It is node,geometry
    #Calculating destination points:
    try:
        for x in source_nodes:
            for y in des_nodes:
                z = get_geometry_from_nodes(y)
                if(z is None):
                    continue

                dist = get_heuristic_value(x[0],y[0])
                heapq.heappush(priority_queue,[dist,x[0]])
                distance[x[0]] = 0

            print("X is " + str(x) + "  " + str(len(priority_queue)))

            while(len(priority_queue)>0):

                u = heapq.heappop(priority_queue)
                print(str(len(priority_queue))+"    "+str(u[1]))
                if(u[1] in visited):
                    continue
                visited.add(u[1])
                edges = get_nodes(u[1])


                for i in edges:
                    if(i in visited):
                        continue
                    #print("Node"+str(type(i)))
                    #print("U[1]"+str(type(u[1])))
                    edge_wt = get_travel_distance(i,u[1])
                    #print("Edge wt"+str(type(edge_wt)))
                    #print("distance "+str(type(distance[u[1]])))
                    fn = distance[u[1]]+edge_wt
                    print("Calculated fn")
                    distance[i]=fn
                    path[i]=u[1]
                    for k in des_nodes:
                        gn = get_heuristic_value(k[0],i)
                        #print("Going to update queue")
                        heapq.heappush(priority_queue,[fn+gn,i])
                        if(gn<0.5):
                            routes.append([x[0],k[0]])
                            if(k not in path):
                                path[k]=[i]
                            flag =1
                            break

                        break
                    if(flag==1):
                        break
                if(flag==1):
                    break

            print("It has found des")
            for i in routes:
                node_point = i[1]
                route = []
                print(i)
                while (node_point != i[0] and (node_point in path)):
                    route.append(node_point)
                    node_point = path[node_point]
                route.append(node_point)
                print("Going to print route")
                for j in route:
                    print((str(j)))

            print("GOing to print attributes")
            print(visited)
            print(distance)
            print(path)
            print(des_nodes)
            break
            del priority_queue[:]  # First element will be fn + gn and second element will be node
            visited.clear()  # Checks if the given nodes is visited
            distance.clear()  # Gets the distance from source
            path.clear()


    except Exception as e:
        print(e)
        exc_type, exc_obj, exc_tb = sys.exc_info()
        fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
        print(exc_type, fname, exc_tb.tb_lineno)
        print("Query Failed")

if __name__=='__main__':
    for i in list_of_points:
        q = get_latitutde_longitude(i[0])
        w = get_lat_long(q[1], q[0])
        final_points.append([i[0], w])
    A_star("Mehrauli","Qutub Minar")
