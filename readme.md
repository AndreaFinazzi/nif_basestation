TODO:
- add visualizer?
- get stuff working... haha
- make docker nicer (docker-compose and X11 forwarding?)

Joshua Spisak <jspisak@andrew.cmu.edu>

Dependencies:
docker
docker-compose (https://docs.docker.com/compose/install/)

Prelims:
bastation ip:
-    address: 10.42.0.<70 - 79>
-    netmask: 255.255.255.0

Note- Leave Gateway empty


Set up Environment:

```
./operations/scripts/bst_docker_build.sh
```

or

```
docker pull iacteams/basestation
```

Then:
```
./operations/scripts/bst_docker_up.sh
./operations/scripts/launch_basestation.sh
```

To open a bash shell inside the container:
```
./operations/scripts/bst_docker_join.sh
```
