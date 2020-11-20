from importlinter.application import output
from importlinter.domain import fields, helpers
from importlinter.domain.contract import Contract, ContractCheck
from importlinter.domain.ports.graph import ImportGraph


class ForbiddenContractDirect(Contract):
    """
    Forbidden contracts check that one set of modules are not imported by another set of modules.
    Indirect imports will also be checked.
    Configuration options:
        - source_modules:    A list of Modules that should not import the forbidden modules.
        - forbidden_modules: A list of Modules that should not be imported by the source modules.
        - ignore_imports:    A set of DirectImports. These imports will be ignored: if the import
                             would cause a contract to be broken, adding it to the set will cause
                             the contract be kept instead. (Optional.)
    """

    type_name = "forbidden_direct"

    source_modules = fields.ListField(subfield=fields.ModuleField())
    forbidden_modules = fields.ListField(subfield=fields.ModuleField())
    ignore_imports = fields.SetField(
        subfield=fields.DirectImportField(), required=False
    )
    allow_indirect_imports = fields.StringField(required=False)

    def check(self, graph: ImportGraph) -> ContractCheck:
        is_kept = True
        invalid_chains = []

        helpers.pop_imports(
            graph, self.ignore_imports if self.ignore_imports else []  # type: ignore
        )

        self._check_all_modules_exist_in_graph(graph)
        self._check_external_forbidden_modules(graph)

        # We only need to check for illegal imports for forbidden modules that are in the graph.
        forbidden_modules_in_graph = [
            m for m in self.forbidden_modules if m.name in graph.modules  # type: ignore
        ]

        for source_module in self.source_modules:  # type: ignore
            for forbidden_module in forbidden_modules_in_graph:
                subpackage_chain_data = {
                    "upstream_module": forbidden_module.name,
                    "downstream_module": source_module.name,
                    "chains": [],
                }

                chains = graph.find_shortest_chains(
                    importer=source_module.name, imported=forbidden_module.name
                )
                if chains:
                    if self._allow_indirect_imports():
                        chains = [chain for chain in chains if len(chain) <= 2]
                        if len(chains) == 0:
                            continue
                    is_kept = False
                    for chain in chains:
                        chain_data = []
                        for importer, imported in [
                            (chain[i], chain[i + 1]) for i in range(len(chain) - 1)
                        ]:
                            import_details = graph.get_import_details(
                                importer=importer, imported=imported
                            )
                            line_numbers = tuple(
                                j["line_number"] for j in import_details
                            )
                            chain_data.append(
                                {
                                    "importer": importer,
                                    "imported": imported,
                                    "line_numbers": line_numbers,
                                }
                            )
                        subpackage_chain_data["chains"].append(chain_data)
                if subpackage_chain_data["chains"]:
                    invalid_chains.append(subpackage_chain_data)

        return ContractCheck(kept=is_kept, metadata={"invalid_chains": invalid_chains})

    def render_broken_contract(self, check: "ContractCheck") -> None:
        count = 0
        for chains_data in check.metadata["invalid_chains"]:
            downstream, upstream = (
                chains_data["downstream_module"],
                chains_data["upstream_module"],
            )
            output.print_error(f"{downstream} is not allowed to import {upstream}:")
            output.new_line()
            count += len(chains_data["chains"])
            for chain in chains_data["chains"]:
                first_line = True
                for direct_import in chain:
                    importer, imported = (
                        direct_import["importer"],
                        direct_import["imported"],
                    )
                    line_numbers = ", ".join(
                        f"l.{n}" for n in direct_import["line_numbers"]
                    )
                    import_string = f"{importer} -> {imported} ({line_numbers})"
                    if first_line:
                        output.print_error(f"-   {import_string}", bold=False)
                        first_line = False
                    else:
                        output.indent_cursor()
                        output.print_error(import_string, bold=False)
                output.new_line()

            output.new_line()

    def _check_all_modules_exist_in_graph(self, graph: ImportGraph) -> None:
        for module in self.source_modules:  # type: ignore
            if module.name not in graph.modules:
                raise ValueError(f"Module '{module.name}' does not exist.")

    def _check_external_forbidden_modules(self, graph: ImportGraph) -> None:
        if (
            self._contains_external_forbidden_modules(graph)
            and not self._graph_was_built_with_externals()
        ):
            raise ValueError(
                "The top level configuration must have include_external_packages=True "
                "when there are external forbidden modules."
            )

    def _contains_external_forbidden_modules(self, graph: ImportGraph) -> bool:
        root_packages = self.session_options["root_packages"]
        return not all(
            m.root_package_name in root_packages for m in self.forbidden_modules  # type: ignore
        )

    def _allow_indirect_imports(self) -> bool:
        return self.allow_indirect_imports and self.allow_indirect_imports in (
            "True",
            "true",
        )

    def _graph_was_built_with_externals(self) -> bool:
        return self.session_options.get("include_external_packages") in ("True", "true")
