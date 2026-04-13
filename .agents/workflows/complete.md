---
description: 
---

=======
description: Finalize an completed task, sum up changes, run checks, and merge.
---
# Task Completion Workflow

When a ticket's implementation is believed to be fully functioning and done, perform these wrap-up steps:

1. **Verify Completion**
   - Verify that the ticket has been completely addressed by cross-referencing all Acceptance Criteria.
   
2. **Quality Checks**
   - Run all relevant tests and verifications (e.g., unit tests, integration tests, linting tools). Ensure no regressions were introduced.

3. **Finalize Documentation**
   - Update any necessary `README.md`, docs, or `.agents/rules/` files that were affected by this feature.
   - Log the precise changes inside `/home/df/data/jflinte/CarlaAir/VIPR_CHANGELOG.md`.
   - Add any necessary information to `.VIPR_README.md`

4. **Commit, Merge, and Push**
   - Commit all progress to the feature branch.
   - Merge the feature/fix branch back into the main environment.
   - Push changes to github

5. **Summary & Cleanup**
   - Provide a concise summary of the overall changes directly to the user.
   - Ask the user if they would like to clean up their working tree (which involves deleting the local feature branch that was just merged).
   - Finally, move the completed ticket markdown file into the `./complete` directory for archiving.
>>>>>>> 54e2d63 (Complete task: setup project and modifications ticket)