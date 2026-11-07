PYTHON  ?= python3
VENV    := .venv
VENV_PY := $(VENV)/bin/python
ARGS    ?=

.PHONY: venv install run list-ports list-channels clean

venv:
	$(PYTHON) -m venv $(VENV)

install: venv
	$(VENV_PY) -m pip install --upgrade pip
	$(VENV_PY) -m pip install -r requirements.txt

run: install
	$(VENV_PY) mss54_logger.py $(ARGS)

list-ports: install
	$(VENV_PY) mss54_logger.py --list-ports

list-channels: install
	$(VENV_PY) mss54_logger.py --list-channels

clean:
	rm -rf $(VENV)
