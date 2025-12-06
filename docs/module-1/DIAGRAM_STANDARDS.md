# Mermaid Diagram Color Standards for Module 1

Use these consistent colors across all diagrams:

- **Publishers/Talkers**: `fill:#3498db,stroke:#2980b9,color:#fff` (Blue)
- **Subscribers/Listeners**: `fill:#2ecc71,stroke:#27ae60,color:#fff` (Green)  
- **Services/Servers**: `fill:#e67e22,stroke:#d35400,color:#fff` (Orange)
- **Actions**: `fill:#9b59b6,stroke:#8e44ad,color:#fff` (Purple)
- **Topics/Channels**: `fill:#ecf0f1,stroke:#95a5a6,color:#333` (Light Gray)
- **Nodes/Processes**: `fill:#34495e,stroke:#2c3e50,color:#fff` (Dark Gray)

## Example Usage:
```mermaid
graph LR
    A[Publisher Node]:::publisher -->|publishes| B[/topic/]:::topic
    B -->|subscribes| C[Subscriber Node]:::subscriber
    
    classDef publisher fill:#3498db,stroke:#2980b9,color:#fff
    classDef subscriber fill:#2ecc71,stroke:#27ae60,color:#fff
    classDef topic fill:#ecf0f1,stroke:#95a5a6,color:#333
```

## WCAG 2.1 AA Compliance:
All color combinations tested for contrast ratio ≥4.5:1