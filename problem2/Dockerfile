FROM aiplanning/planutils:latest

# Install solvers and tools
RUN planutils install -y val
RUN planutils install -y ff
RUN planutils install -y metric-ff
RUN planutils install -y enhsp
RUN planutils install -y popf
RUN planutils install -y optic
RUN planutils install -y tfd
RUN planutils install -y downward

# Modify the configuration file to enable hostfs, i.e. use the host file system
RUN perl -pi.bak -e "s/mount hostfs = no/mount hostfs = yes/g" /etc/apptainer/apptainer.conf


CMD ["/bin/bash"]