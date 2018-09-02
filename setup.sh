#!/bin/bash

mkdir venv
virtualenv venv
source venv/bin/activate

pip install -r requirements.txt