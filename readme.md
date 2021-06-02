TODO:
- add visualizer?
- get stuff working... haha
- make docker nicer (docker-compose and X11 forwarding?)

Joshua Spisak <jspisak@andrew.cmu.edu>

Dependencies:
docker
docker-compose (pip3 install --upgrade pip setuptools && pip3 install --upgrade docker-compose)


Do Stuff:
./operations/scripts/bst_docker_build.sh
./operations/scripts/bst_docker_up.sh
./operations/scripts/bst_docker_join.sh

after the join you are in docker do:
tmuxp load launch_joy.yaml