.PHONY: all
all: ##KEEP ME AT THE TOP
	make test && make behave

.PHONY: unittest
unittest:
	./scripts/run_tests.sh

.PHONY: test
test:
	pipenv run py.test

.PHONY: behave
behave:
	pipenv run behave

.PHONY: init-pyre
init-pyre:
	cd ./catkin/src/ && pipenv run pyre init

.PHONY: type-check
type-check:
	cd ./catkin/src/ && pipenv run pyre check