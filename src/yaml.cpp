#include "pmod_ros/yaml.hpp"

#define INDENT "  "
#define STRVAL(x) ((x) ? (char*)(x) : "")

void indent(int level)
{
    int i;
    for (i = 0; i < level; i++) {
        printf("%s", INDENT);
    }
}

void print_event(yaml_event_t *event) {
    static int level = 0;

    switch (event->type) {
    case YAML_NO_EVENT:
        indent(level);
        printf("no-event (%d)\n", event->type);
        break;
    case YAML_STREAM_START_EVENT:
        indent(level++);
        printf("stream-start-event (%d)\n", event->type);
        break;
    case YAML_STREAM_END_EVENT:
        indent(--level);
        printf("stream-end-event (%d)\n", event->type);
        break;
    case YAML_DOCUMENT_START_EVENT:
        indent(level++);
        printf("document-start-event (%d)\n", event->type);
        break;
    case YAML_DOCUMENT_END_EVENT:
        indent(--level);
        printf("document-end-event (%d)\n", event->type);
        break;
    case YAML_ALIAS_EVENT:
        indent(level);
        printf("alias-event (%d)\n", event->type);
        break;
    case YAML_SCALAR_EVENT:
        indent(level);
        // printf("scalar-event (%d) = {value=\"%s\", length=%d}\n",
        //        event->type,
        //        STRVAL(event->data.scalar.value),
        //        (int)event->data.scalar.length);
        break;
    case YAML_SEQUENCE_START_EVENT:
        indent(level++);
        printf("sequence-start-event (%d)\n", event->type);
        break;
    case YAML_SEQUENCE_END_EVENT:
        indent(--level);
        printf("sequence-end-event (%d)\n", event->type);
        break;
    case YAML_MAPPING_START_EVENT:
        indent(level++);
        printf("mapping-start-event (%d)\n", event->type);
        break;
    case YAML_MAPPING_END_EVENT:
        indent(--level);
        printf("mapping-end-event (%d)\n", event->type);
        break;
    }
    if (level < 0) {
        fprintf(stderr, "indentation underflow!\n");
        level = 0;
    }
}

Any parse_mapping(yaml_parser_t *parser, yaml_event_t *event)
{
    std::map<std::string, Any> map;

    bool done = false;
    while (!done) {
        if (!yaml_parser_parse(parser, event)) {
            throw std::runtime_error("Parse Error");
        }

        switch (event->type) {
            case YAML_MAPPING_START_EVENT: {
                yaml_event_delete(event);
                parse_mapping(parser, event);
            } break;
            case YAML_MAPPING_END_EVENT: {
                done = true;
            } break;
        }
        yaml_event_delete(event);
    }
}

Any parse_yaml(const std::string &path)
{
    yaml_parser_t parser;
    yaml_event_t event;
    FILE *file;
    bool done = false;
    file = fopen(path.c_str(), "rb");
    yaml_parser_initialize(&parser);
    yaml_parser_set_input_file(&parser, file);

    while (!done) {
        if (!yaml_parser_parse(&parser, &event)) {
            std::cerr << "ERROR" << std::endl;
            break;
        }
        print_event(&event);

        switch (event.type) {
            case YAML_SCALAR_EVENT: {
                std::string value((char*)event.data.scalar.value);
                std::cout << value << std::endl;
            } break;
            case YAML_STREAM_END_EVENT: {
                done = true;
            } break;
        }

        yaml_event_delete(&event);
    }

    fclose(file);

    return 0;
}
