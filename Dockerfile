FROM golang:latest AS go-mavgen

RUN go get -u github.com/asmyasnikov/go-mavlink/mavgen

FROM python:buster AS py-mavgen

RUN pip install pymavlink

RUN pip install https://github.com/pyinstaller/pyinstaller/archive/develop.tar.gz

RUN pyinstaller /usr/local/bin/mavgen.py --onefile --name mavgen.py --add-data $(python -c "import site; print(site.getsitepackages()[0])")/pymavlink/generator:pymavlink/generator

FROM debian:buster-slim

COPY --from=go-mavgen /go/bin/mavgen /usr/bin

COPY --from=py-mavgen /dist/mavgen.py /usr/bin

RUN apt update && \
    apt install -y \
        uuid-runtime \
        openssh-client \
        git \
        wget \
        libxslt1-dev  
