## Basestation

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

To close the base station, type:
```
Ctrl-B, :kill-session
```
and press Enter.

Troubleshoot:

Issue #1 :  docker.errors.DockerException: Error while fetching server API version: ('Connection aborted.', PermissionError(13, 'Permission denied'))

```
sudo chmod 666 /var/run/docker.sock
```

## Use battery monitor
- Launch a separate terminal on a pitlane laptop
- 
