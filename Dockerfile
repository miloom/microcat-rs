FROM arm64v8/ubuntu:22.04

RUN apt-get update
RUN apt-get install -y bash
RUN rm -rf /var/lib/apt/lists/*

CMD ["/bin/bash"]
