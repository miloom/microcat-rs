#FROM --platform=$BUILDPLATFORM arm64v8/ubuntu:22.04 AS build
#ARG TARGETPLATFORM
#ARG BUILDPLATFORM
#RUN echo "I am running on $BUILDPLATFORM, building for $TARGETPLATFORM" > /log
FROM arm64v8/ubuntu:22.04

# COPY --from=build /log /log

RUN apt-get update
RUN apt-get install -y bash
RUN rm -rf /var/lib/apt/lists/*

CMD ["/bin/bash"]
