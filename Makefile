.PHONY: install test test-notebooks lint format notebooks research catalog app validate clean

install:
	pip install -e ".[development]"

test:
	python -m pytest tests -q --ignore=tests/notebook_validation

test-notebooks:
	python -m pytest tests/notebook_validation -q

lint:
	ruff check src tests scripts tools

format:
	ruff format src tests scripts tools

# build notebooks from specs, then execute them from a clean kernel
notebooks:
	python tools/build_notebooks.py
	python tools/run_notebooks.py

research:
	python tools/build_research.py

catalog:
	python tools/build_catalog.py

validate:
	python scripts/validate_results.py
	python scripts/validate_catalog.py
	python scripts/check_links.py

app:
	python tools/build_app.py

clean:
	find . -name __pycache__ -type d -prune -exec rm -rf {} +
	rm -rf .pytest_cache .ruff_cache app/dist
