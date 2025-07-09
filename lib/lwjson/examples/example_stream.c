#include <stdio.h>
#include "lwjson/lwjson.h"

/* Test string to parser */
static const char* json_str = "{\"k1\":\"v1\",\"k2\":[true, false]}";

/* LwJSON stream parser */
static lwjson_stream_parser_t stream_parser;

/**
 * \brief           Callback function for various events
 * \param           jsp: JSON stream parser object
 * \param           type: Event type
 */
static void
prv_example_callback_func(lwjson_stream_parser_t* jsp, lwjson_stream_type_t type) {
    /* Get a value corresponsing to "k1" key */
    if (jsp->stack_pos >= 2 /* Number of stack entries must be high */
        && lwjson_stack_seq_2(jsp, 0, OBJECT, KEY) && strcmp(jsp->stack[1].meta.name, "k1") == 0) {
        printf("Got key '%s' with value '%s'\r\n", jsp->stack[1].meta.name, jsp->data.str.buff);
    }
    (void)type;
}

/* Parse JSON */
void
example_stream_run(void) {
    lwjsonr_t res;
    printf("\r\n\r\nParsing stream\r\n");
    lwjson_stream_init(&stream_parser, prv_example_callback_func);

    /* Demonstrate as stream inputs */
    for (const char* c = json_str; *c != '\0'; ++c) {
        res = lwjson_stream_parse(&stream_parser, *c);
        if (res == lwjsonSTREAMINPROG) {
        } else if (res == lwjsonSTREAMWAITFIRSTCHAR) {
            printf("Waiting first character\r\n");
        } else if (res == lwjsonSTREAMDONE) {
            printf("Done\r\n");
        } else {
            printf("Error\r\n");
            break;
        }
    }
    printf("Parsing completed\r\n");
}
