.PHONY: unittest
unittest:
	./scripts/run_tests.sh

.PHONY: test
test:
	pipenv run py.test

.PHONY: behave
behave:
	pipenv run behave