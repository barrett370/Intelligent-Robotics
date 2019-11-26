.PHONY: all
all: ##KEEP ME AT THE TOP
	make test && make behave && make type-check

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
	cd ./vision && pipenv run pyre init

.PHONY: type-check
type-check:
	cd ./vision && pipenv run pyre check

.PHONY: gen-resources
generate:
	cd ./speech && mkdir -p ./resources/snippets && pipenv run python gen-speech-snippets.py

