# Data Model: Chapter on AI-Native Software Development

**Purpose**: To define the key entities and their attributes for the chapter creation feature.

## Entity Relationship Diagram (Conceptual)

```
[Chapter] --(contains-many)--> [Citation]
[Citation] --(references-one)--> [Source]
```

## Entity Definitions

### Chapter

The primary content artifact being created.

| Attribute | Type | Description | Constraints |
| :--- | :--- | :--- | :--- |
| Title | String | The title of the chapter. | Required |
| Abstract | Text | A short summary of the chapter. | Required |
| Body | Markdown | The main content of the chapter, including all sections. | 5,000-7,000 words |
| References | List<Citation> | The list of all citations used in the chapter. | Min 15 citations |
| Status | Enum | The current state of the chapter (e.g., Draft, In Review, Final). | |

### Citation

A formatted reference connecting a claim within the chapter to a specific source.

| Attribute | Type | Description | Constraints |
| :--- | :--- | :--- | :--- |
| InTextID | String | The identifier used in the body text (e.g., "Author, 2025"). | Required, APA 7th Ed. format |
| FullReference | String | The full bibliographic entry for the source. | Required, APA 7th Ed. format |
| SourceID | FK | A foreign key linking to the `Source` entity. | Required |

### Source

The original academic paper, book, or article from which information is derived.

| Attribute | Type | Description | Constraints |
| :--- | :--- | :--- | :--- |
| SourceID | PK | Unique identifier for the source. | Required |
| Title | String | The title of the work. | Required |
| Authors | List<String> | The authors of the work. | Required |
| PublicationDate | Date | The date the work was published. | Required |
| SourceType | Enum | The type of source (e.g., Peer-Reviewed Journal, Book, Article). | Required |
