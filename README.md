# HanS
Visual sensemaking repo for the Health &amp; Safety robot inspector (HanS)

## Dependencies

* ROS distro: galactic 
* [ROS2](https://docs.ros.org/en/galactic/Installation.html) 
  [Instructions for setting up a ROS2 workspace](https://docs.ros.org/en/galactic/Tutorials/Workspace/Creating-A-Workspace.html). 
  Then, this repo can be cloned under the ```src``` folder. 
  
* Bounding box detection via OpenCV. 
  Object classification via USB, through a Coral Edge TPU.
  Setting up the Edge TPU accelerator and Pycoral library: [https://coral.ai/docs/accelerator/get-started](https://coral.ai/docs/accelerator/get-started)

* [Open3D Python library](http://www.open3d.org/docs/0.12.0/introduction.html) v.0.12 for 3D data processing

* ```apt install ros-galactic-sensor-msgs-py```

* Set up a PostgreSQL database, with the PostGIS and SFCGAL extensions enabled. 

    - Install [PostgreSQL 12](https://www.postgresql.org/download/linux/ubuntu/)
      - `apt-get install postgresql-12`
    - Build [CGAL](https://github.com/CGAL/cgal)>= 5.3 from source
      - `wget https://github.com/CGAL/cgal/archive/refs/tags/v5.3.2.zip`
      - `unzip v5.3.2.zip`
      - `cd cgal-5.3.2`
      - `mkdir build && cd build`
      - `cmake .. `
      - `sudo make install`
    - Build [SFCGAL](https://gitlab.com/Oslandia/SFCGAL) from source
      - `wget https://gitlab.com/Oslandia/SFCGAL/-/archive/v1.4.1/SFCGAL-v1.4.1.zip`
      - `unzip SFCGAL-v1.4.1.zip`
      - `cd SFCGAL-v1.4.1`
      - `cmake . && make && sudo make install`
    - Install [PostGIS](https://postgis.net/install/)
      - `sudo apt-get install postgresql-12-postgis-3`
    - [PgAdmin 4](https://www.pgadmin.org/download/pgadmin-4-apt/) provides a helpful interface to visualize and query your DB.
      - `sudo apt install curl`
      - `curl -fsS https://www.pgadmin.org/static/packages_pgadmin_org.pub | sudo gpg --dearmor -o /usr/share/keyrings/packages-pgadmin-org.gpg`
      - `sudo sh -c 'echo "deb [signed-by=/usr/share/keyrings/packages-pgadmin-org.gpg] https://ftp.postgresql.org/pub/pgadmin/pgadmin4/apt/$(lsb_release -cs) pgadmin4 main" > /etc/apt/sources.list.d/pgadmin4.list && apt update'`
      - `sudo apt install pgadmin4`
    - Setup a PostgreSQL user:
      - `sudo -i -u postgres`
      - `psql`
      - `ALTER USER postgres PASSWORD 'password';`
    - Open PgAdmin and create a server by clicking `Add new Server` in the home dashboard. The server name must be `gis_database`, the hostname `localhost`, and the owner `postgres` user. 
    - Select the DB `` and click on the `Query Tool` at the top of the sidebar.
      
    <!---Once PgAdmin is up and running, create a database that you can access with admin privileges. 
    By default, the code expects a DB user with the same name of the $USER variable on Linux with admin privileges.  
    
    To enable postGIS and SFCGAL on your DB, run the following commands from the query editor in PgAdmin:
      ```
    CREATE EXTENSION postgis;
    CREATE EXTENSION postgis_sfcgal;
    ```
    -->
    The schema of the DB is documented under ```./postgresql/create_tables.sql```


## Additional Database setup notes
Instructions in ```./postgresql``` folder. 
Create tables first through the queries in the ```create_queries.sql``` file.
Apply triggers to queries as explained in the ```trigger_functions.md``` file.

## Running on your own map
The script ```postgresql/draw_walls.py``` can be used for annotating the walls of an input map.
The image of the map and configuration parameters (map origin, and resolution), should 
follow the same format of the files provided under ```./map```. 
