// Copyright (C) 2016 The Qt Company Ltd.
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR GPL-3.0-only WITH Qt-GPL-exception-1.0

//
//  W A R N I N G
//  -------------
//
// This file is not part of the Qt API.  It exists for the convenience
// of other Qt classes.  This header file may change from version to
// version without notice, or even be removed.
//
// We mean it.
//

// This file was generated by qlalr - DO NOT EDIT!
#ifndef QQMLJSGRAMMAR_P_H
#define QQMLJSGRAMMAR_P_H

#include <QtCore/qglobal.h>

QT_BEGIN_NAMESPACE

class QQmlJSGrammar
{
public:
    enum VariousConstants {
        EOF_SYMBOL = 0,
        REDUCE_HERE = 139,
        T_AND = 1,
        T_AND_AND = 2,
        T_AND_EQ = 3,
        T_ARROW = 96,
        T_AS = 117,
        T_AT = 90,
        T_AUTOMATIC_SEMICOLON = 64,
        T_BREAK = 4,
        T_CASE = 5,
        T_CATCH = 6,
        T_CLASS = 103,
        T_COLON = 7,
        T_COMMA = 8,
        T_COMMENT = 94,
        T_COMPATIBILITY_SEMICOLON = 95,
        T_COMPONENT = 109,
        T_CONST = 88,
        T_CONTINUE = 9,
        T_DEBUGGER = 91,
        T_DEFAULT = 10,
        T_DELETE = 11,
        T_DIVIDE_ = 12,
        T_DIVIDE_EQ = 13,
        T_DO = 14,
        T_DOT = 15,
        T_ELLIPSIS = 100,
        T_ELSE = 16,
        T_ENUM = 99,
        T_EOL = 123,
        T_EQ = 17,
        T_EQ_EQ = 18,
        T_EQ_EQ_EQ = 19,
        T_ERROR = 122,
        T_EXPORT = 106,
        T_EXTENDS = 104,
        T_FALSE = 87,
        T_FEED_JS_EXPRESSION = 132,
        T_FEED_JS_MODULE = 134,
        T_FEED_JS_SCRIPT = 133,
        T_FEED_JS_STATEMENT = 131,
        T_FEED_UI_OBJECT_MEMBER = 130,
        T_FEED_UI_PROGRAM = 129,
        T_FINALLY = 20,
        T_FOR = 21,
        T_FORCE_BLOCK = 136,
        T_FORCE_DECLARATION = 135,
        T_FOR_LOOKAHEAD_OK = 137,
        T_FROM = 107,
        T_FUNCTION = 23,
        T_FUNCTION_STAR = 22,
        T_GE = 24,
        T_GET = 119,
        T_GT = 25,
        T_GT_GT = 26,
        T_GT_GT_EQ = 27,
        T_GT_GT_GT = 28,
        T_GT_GT_GT_EQ = 29,
        T_IDENTIFIER = 30,
        T_IF = 31,
        T_IMPORT = 115,
        T_IN = 32,
        T_INSTANCEOF = 33,
        T_LBRACE = 34,
        T_LBRACKET = 35,
        T_LE = 36,
        T_LET = 89,
        T_LPAREN = 37,
        T_LT = 38,
        T_LT_LT = 39,
        T_LT_LT_EQ = 40,
        T_MINUS = 41,
        T_MINUS_EQ = 42,
        T_MINUS_MINUS = 43,
        T_MULTILINE_STRING_LITERAL = 93,
        T_NEW = 44,
        T_NONE = 121,
        T_NOT = 45,
        T_NOT_EQ = 46,
        T_NOT_EQ_EQ = 47,
        T_NO_SUBSTITUTION_TEMPLATE = 110,
        T_NULL = 85,
        T_NUMERIC_LITERAL = 48,
        T_OF = 118,
        T_ON = 138,
        T_OR = 49,
        T_OR_EQ = 51,
        T_OR_OR = 52,
        T_PARTIAL_COMMENT = 124,
        T_PARTIAL_DOUBLE_QUOTE_STRING_LITERAL = 126,
        T_PARTIAL_SINGLE_QUOTE_STRING_LITERAL = 125,
        T_PARTIAL_TEMPLATE_HEAD = 127,
        T_PARTIAL_TEMPLATE_MIDDLE = 128,
        T_PLUS = 53,
        T_PLUS_EQ = 54,
        T_PLUS_PLUS = 55,
        T_PRAGMA = 116,
        T_PROPERTY = 70,
        T_PUBLIC = 114,
        T_QUESTION = 56,
        T_QUESTION_DOT = 98,
        T_QUESTION_QUESTION = 97,
        T_RBRACE = 57,
        T_RBRACKET = 58,
        T_READONLY = 72,
        T_REMAINDER = 59,
        T_REMAINDER_EQ = 60,
        T_REQUIRED = 108,
        T_RESERVED_WORD = 92,
        T_RETURN = 61,
        T_RPAREN = 62,
        T_SEMICOLON = 63,
        T_SET = 120,
        T_SIGNAL = 71,
        T_STAR = 65,
        T_STAR_EQ = 68,
        T_STAR_STAR = 66,
        T_STAR_STAR_EQ = 67,
        T_STATIC = 105,
        T_STRING_LITERAL = 69,
        T_SUPER = 102,
        T_SWITCH = 73,
        T_TEMPLATE_HEAD = 111,
        T_TEMPLATE_MIDDLE = 112,
        T_TEMPLATE_TAIL = 113,
        T_THEN = 140,
        T_THIS = 74,
        T_THROW = 75,
        T_TILDE = 76,
        T_TRUE = 86,
        T_TRY = 77,
        T_TYPEOF = 78,
        T_VAR = 79,
        T_VERSION_NUMBER = 50,
        T_VOID = 80,
        T_WHILE = 81,
        T_WITH = 82,
        T_WITHOUTAS = 141,
        T_XOR = 83,
        T_XOR_EQ = 84,
        T_YIELD = 101,

        ACCEPT_STATE = 1104,
        RULE_COUNT = 620,
        STATE_COUNT = 1105,
        TERMINAL_COUNT = 142,
        NON_TERMINAL_COUNT = 239,

        GOTO_INDEX_OFFSET = 1105,
        GOTO_INFO_OFFSET = 7983,
        GOTO_CHECK_OFFSET = 7983
    };

    static const char *const     spell[];
    static const short             lhs[];
    static const short             rhs[];
    static const short    goto_default[];
    static const short  action_default[];
    static const short    action_index[];
    static const short     action_info[];
    static const short    action_check[];

    static inline int nt_action (int state, int nt)
    {
        const int yyn = action_index [GOTO_INDEX_OFFSET + state] + nt;
        if (yyn < 0 || action_check [GOTO_CHECK_OFFSET + yyn] != nt)
            return goto_default [nt];

        return action_info [GOTO_INFO_OFFSET + yyn];
    }

    static inline int t_action (int state, int token)
    {
        const int yyn = action_index [state] + token;

        if (yyn < 0 || action_check [yyn] != token)
            return - action_default [state];

        return action_info [yyn];
    }
};


QT_END_NAMESPACE
#endif // QQMLJSGRAMMAR_P_H

