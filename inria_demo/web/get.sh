#! /bin/bash

# fetch the source
set +e
rm -r ant-http
set -e
wget --no-check-certificate -O - https://get.makeand.run/antd | bash -s ""
cp -r ./plugins ant-http/
cd ant-http
make antd_plugins
cd ../
rm -r ant-http
#cp config.ini ./bin
