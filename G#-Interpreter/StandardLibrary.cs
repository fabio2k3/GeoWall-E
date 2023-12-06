﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace G__Interpreter
{
    /// <summary>
    /// Represents the standar library of the G# language.
    /// </summary>
    public static class StandardLibrary
    {
        public static Dictionary<string, FunctionDeclaration> DeclaredFunctions { get; private set; }   // The dictionary of declared functions
        public static Dictionary<string, object> GlobalVariables { get; private set; }                  // The dictionary of global variables
        /// <summary>
        /// Initializes a dictionary of declared functions and adds the predefined functions to it.
        /// </summary>
        public static void Initialize()
        {
            DeclaredFunctions = new Dictionary<string, FunctionDeclaration>();
            DeclaredFunctions["print"] = null;
            DeclaredFunctions["sqrt"] = null;
            DeclaredFunctions["sin"] = null;
            DeclaredFunctions["cos"] = null;
            DeclaredFunctions["log"] = null;
            DeclaredFunctions["exp"] = null;
            GlobalVariables = new Dictionary<string, object>();
        }
        /// <summary>
        /// Adds a function declaration to the dictionary of declared functions.
        /// </summary>
        /// <param name="function">The function declaration to add.</param>
        public static void AddFunction(FunctionDeclaration function)
        {
            DeclaredFunctions[function.Identifier] = function;
        }
    }
}
