# CarlaAir Agent Workflows

This document lists the available automated workflows for the CarlaAir project.

| Workflow | Command | Description |
| :--- | :--- | :--- |
| **Ticket Initialization** | `@/ticket` | Start work on a new ticket, create branch, and setup context. |
| **Task Completion** | `@/complete` | Verify, document, merge, and archive a completed ticket. |
| **Headless Server** | `@/headless_server` | Guidelines for starting the CarlaAir server in headless mode. |

## Available Workflow Files
- [ticket.md](file:///home/df/data/jflinte/CarlaAir/.agents/workflows/ticket.md)
- [complete.md](file:///home/df/data/jflinte/CarlaAir/.agents/workflows/complete.md)
- [headless_server.md](file:///home/df/data/jflinte/CarlaAir/.agents/workflows/headless_server.md)

## Core Agent Skills

### 1. Environment & Execution (UV)
The agent MUST always prioritize `uv` for environment management and script execution. 
- **Rule**: Never use system python or conda unless explicitly requested.
- **Action**: Prepend execution commands with `uv run` or utilize the `.venv/bin/python` path to ensure dependency alignment.
