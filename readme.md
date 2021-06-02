TODO:
- add visualizer?
- get stuff working... haha
- make docker nicer (docker-compose and X11 forwarding?)

Joshua Spisak <jspisak@andrew.cmu.edu>

Dependencies:
docker
docker-compose (pip3 install --upgrade pip setuptools && pip3 install --upgrade docker-compose)

Prelims:
bastation ip:
    address: 10.42.0.<70 - 79>
    netmask: 255.255.255.0

Set up Environment:
./operations/scripts/bst_docker_build.sh or docker pull iacteams/basestation
./operations/scripts/bst_docker_up.sh
./operations/scripts/bst_docker_join.sh

after the join you are in docker do:
tmuxp load launch_joy.yaml