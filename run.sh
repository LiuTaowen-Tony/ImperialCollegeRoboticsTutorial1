#! /bin/bash

source venv/bin/activate
mypy main.py && main.py
