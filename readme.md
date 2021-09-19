## Basestation

TODO:
- add visualizer?
- get stuff working... haha
- make docker nicer (docker-compose and X11 forwarding?)

Joshua Spisak <jspisak@andrew.cmu.edu>

Dependencies:
docker
docker-compose (https://docs.docker.com/compose/install/)

Configuration:
- Update `operations/scripts/bst_fm.sh` to reflect your vehicle #.
- ie. `ROS_DOMAIN_ID=N2 RUST_LOG=debug zenoh-bridge-dds -d N2 -m peer -e tcp/10.42.N.200:7447` where `N` is your vehicle #

- Update `operations/config/launch_basestation.yaml` to point to the correct IP address of your Adlink
- ie. `10.42.N.200` where `N` is your vehicle #

- Update `operations/dockerfiles/basestation.Dockerfile` to use the correct `ROS_DOMAIN_ID`
- ie. `RUN echo "export ROS_DOMAIN_ID=N2" >> ~/.bashrc` where `N` is your vehicle #

- Update the `remote_spoof*`, `shutdown.sh` scripts to use the correct vehicle IP address


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
