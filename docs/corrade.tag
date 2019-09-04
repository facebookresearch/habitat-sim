<?xml version='1.0' encoding='UTF-8' standalone='yes' ?>
<tagfile>
  <compound kind="file">
    <name>Array.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/Containers/</path>
    <filename>Array_8h</filename>
    <includes id="ArrayView_8h" name="ArrayView.h" local="yes" imported="no">Corrade/Containers/ArrayView.h</includes>
    <includes id="Tags_8h" name="Tags.h" local="yes" imported="no">Corrade/Containers/Tags.h</includes>
    <class kind="class">Corrade::Containers::Array</class>
    <namespace>Corrade</namespace>
    <namespace>Corrade::Containers</namespace>
    <member kind="function">
      <type>ArrayView&lt; T &gt;</type>
      <name>arrayView</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>a57f8d4e0a25951f539461539c90d4e25</anchor>
      <arglist>(Array&lt; T, D &gt; &amp;array)</arglist>
    </member>
    <member kind="function">
      <type>ArrayView&lt; const T &gt;</type>
      <name>arrayView</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>a4258433529f847d3e6ed10442f6b9b8b</anchor>
      <arglist>(const Array&lt; T, D &gt; &amp;array)</arglist>
    </member>
    <member kind="function">
      <type>ArrayView&lt; U &gt;</type>
      <name>arrayCast</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>ad5f0615634b2553b0cb304aa58da3347</anchor>
      <arglist>(Array&lt; T, D &gt; &amp;array)</arglist>
    </member>
    <member kind="function">
      <type>ArrayView&lt; const U &gt;</type>
      <name>arrayCast</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>a9a1ca1efe52f9930f1b31b9feab5a145</anchor>
      <arglist>(const Array&lt; T, D &gt; &amp;array)</arglist>
    </member>
    <member kind="function">
      <type>std::size_t</type>
      <name>arraySize</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>af8a4e6b17dca34a5eb82b6a8da8c398b</anchor>
      <arglist>(const Array&lt; T &gt; &amp;view)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>ArrayView.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/Containers/</path>
    <filename>ArrayView_8h</filename>
    <includes id="Containers_8h" name="Containers.h" local="yes" imported="no">Corrade/Containers/Containers.h</includes>
    <includes id="Assert_8h" name="Assert.h" local="yes" imported="no">Corrade/Utility/Assert.h</includes>
    <class kind="class">Corrade::Containers::ArrayView</class>
    <class kind="class">Corrade::Containers::ArrayView&lt; void &gt;</class>
    <class kind="class">Corrade::Containers::ArrayView&lt; const void &gt;</class>
    <class kind="class">Corrade::Containers::StaticArrayView</class>
    <namespace>Corrade</namespace>
    <namespace>Corrade::Containers</namespace>
    <member kind="function">
      <type>constexpr ArrayView&lt; T &gt;</type>
      <name>arrayView</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>a32ef8daa386aee742e5f39fc5b1516fd</anchor>
      <arglist>(T *data, std::size_t size)</arglist>
    </member>
    <member kind="function">
      <type>constexpr ArrayView&lt; T &gt;</type>
      <name>arrayView</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>ac8edb9cd0a74f5a0da2ef9c6d03f9ef2</anchor>
      <arglist>(T(&amp;data)[size])</arglist>
    </member>
    <member kind="function">
      <type>constexpr ArrayView&lt; T &gt;</type>
      <name>arrayView</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>ac450a49515ff20058552c732542b1b13</anchor>
      <arglist>(StaticArrayView&lt; size, T &gt; view)</arglist>
    </member>
    <member kind="function">
      <type>constexpr ArrayView&lt; T &gt;</type>
      <name>arrayView</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>a3a43320e58651eafa9774e04c68545e5</anchor>
      <arglist>(ArrayView&lt; T &gt; view)</arglist>
    </member>
    <member kind="function">
      <type>constexpr U</type>
      <name>arrayView</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>ade5f07807e9559c80c45e3c314299b91</anchor>
      <arglist>(T &amp;&amp;other)</arglist>
    </member>
    <member kind="function">
      <type>ArrayView&lt; U &gt;</type>
      <name>arrayCast</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>a8e6435208f2cda38505084a930a3e9ec</anchor>
      <arglist>(ArrayView&lt; T &gt; view)</arglist>
    </member>
    <member kind="function">
      <type>constexpr std::size_t</type>
      <name>arraySize</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>a19e087600ec3a4f81931386ee28e39ef</anchor>
      <arglist>(ArrayView&lt; T &gt; view)</arglist>
    </member>
    <member kind="function">
      <type>constexpr std::size_t</type>
      <name>arraySize</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>a9e4a13e421a34a19eec002ed65bda582</anchor>
      <arglist>(StaticArrayView&lt; size_, T &gt;)</arglist>
    </member>
    <member kind="function">
      <type>constexpr std::size_t</type>
      <name>arraySize</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>aaf76e645cbe62b360a0f13de4dff2837</anchor>
      <arglist>(T(&amp;)[size_])</arglist>
    </member>
    <member kind="function">
      <type>constexpr StaticArrayView&lt; size, T &gt;</type>
      <name>staticArrayView</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>a58d7bb8d46f44949b57b226485db4a9b</anchor>
      <arglist>(T *data)</arglist>
    </member>
    <member kind="function">
      <type>constexpr StaticArrayView&lt; size, T &gt;</type>
      <name>staticArrayView</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>a3cf428b256a5ed8941145e4d15ae8b45</anchor>
      <arglist>(T(&amp;data)[size])</arglist>
    </member>
    <member kind="function">
      <type>constexpr StaticArrayView&lt; size, T &gt;</type>
      <name>staticArrayView</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>aeb96550df22942b8379e77d3942c88c4</anchor>
      <arglist>(StaticArrayView&lt; size, T &gt; view)</arglist>
    </member>
    <member kind="function">
      <type>constexpr U</type>
      <name>staticArrayView</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>a2f1123798b8bee4ebadf785646108c98</anchor>
      <arglist>(T &amp;&amp;other)</arglist>
    </member>
    <member kind="function">
      <type>StaticArrayView&lt; size *sizeof(T)/sizeof(U), U &gt;</type>
      <name>arrayCast</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>aee41b58bdb45963c476ce7554dda2290</anchor>
      <arglist>(StaticArrayView&lt; size, T &gt; view)</arglist>
    </member>
    <member kind="function">
      <type>StaticArrayView&lt; size *sizeof(T)/sizeof(U), U &gt;</type>
      <name>arrayCast</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>aee9c43d476128f05403e638ce539c90d</anchor>
      <arglist>(T(&amp;data)[size])</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>ArrayViewStl.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/Containers/</path>
    <filename>ArrayViewStl_8h</filename>
    <includes id="ArrayView_8h" name="ArrayView.h" local="yes" imported="no">Corrade/Containers/ArrayView.h</includes>
  </compound>
  <compound kind="file">
    <name>ArrayViewStlSpan.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/Containers/</path>
    <filename>ArrayViewStlSpan_8h</filename>
    <includes id="ArrayView_8h" name="ArrayView.h" local="yes" imported="no">Corrade/Containers/ArrayView.h</includes>
  </compound>
  <compound kind="file">
    <name>Containers.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/Containers/</path>
    <filename>Containers_8h</filename>
    <namespace>Corrade</namespace>
    <namespace>Corrade::Containers</namespace>
  </compound>
  <compound kind="file">
    <name>EnumSet.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/Containers/</path>
    <filename>EnumSet_8h</filename>
    <includes id="Containers_8h" name="Containers.h" local="yes" imported="no">Corrade/Containers/Containers.h</includes>
    <includes id="Tags_8h" name="Tags.h" local="yes" imported="no">Corrade/Containers/Tags.h</includes>
    <class kind="class">Corrade::Containers::EnumSet</class>
    <namespace>Corrade</namespace>
    <namespace>Corrade::Containers</namespace>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_ENUMSET_OPERATORS</name>
      <anchorfile>EnumSet_8h.html</anchorfile>
      <anchor>abea606a97fd1f8321cf005b33239bff0</anchor>
      <arglist>(class)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_ENUMSET_FRIEND_OPERATORS</name>
      <anchorfile>EnumSet_8h.html</anchorfile>
      <anchor>a8d5e08f0c65922f00b53c38ccfcaa99a</anchor>
      <arglist>(class)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>EnumSet.hpp</name>
    <path>/home/mosra/Code/corrade/src/Corrade/Containers/</path>
    <filename>EnumSet_8hpp</filename>
    <includes id="EnumSet_8h" name="EnumSet.h" local="yes" imported="no">Corrade/Containers/EnumSet.h</includes>
    <includes id="Debug_8h" name="Debug.h" local="yes" imported="no">Corrade/Utility/Debug.h</includes>
    <namespace>Corrade</namespace>
    <namespace>Corrade::Containers</namespace>
    <member kind="function">
      <type>Utility::Debug &amp;</type>
      <name>enumSetDebugOutput</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>a12c4ea794fa62c56969b1311e2b5c21b</anchor>
      <arglist>(Utility::Debug &amp;debug, EnumSet&lt; T, fullValue &gt; value, const char *empty, std::initializer_list&lt; T &gt; enums)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>LinkedList.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/Containers/</path>
    <filename>LinkedList_8h</filename>
    <includes id="Containers_8h" name="Containers.h" local="yes" imported="no">Corrade/Containers/Containers.h</includes>
    <includes id="Assert_8h" name="Assert.h" local="yes" imported="no">Corrade/Utility/Assert.h</includes>
    <class kind="class">Corrade::Containers::LinkedList</class>
    <class kind="class">Corrade::Containers::LinkedListItem</class>
    <namespace>Corrade</namespace>
    <namespace>Corrade::Containers</namespace>
  </compound>
  <compound kind="file">
    <name>Optional.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/Containers/</path>
    <filename>Optional_8h</filename>
    <includes id="Tags_8h" name="Tags.h" local="yes" imported="no">Corrade/Containers/Tags.h</includes>
    <includes id="Assert_8h" name="Assert.h" local="yes" imported="no">Corrade/Utility/Assert.h</includes>
    <includes id="Debug_8h" name="Debug.h" local="yes" imported="no">Corrade/Utility/Debug.h</includes>
    <class kind="struct">Corrade::Containers::NullOptT</class>
    <class kind="class">Corrade::Containers::Optional</class>
    <namespace>Corrade</namespace>
    <namespace>Corrade::Containers</namespace>
    <member kind="function">
      <type>Optional&lt; typename std::decay&lt; T &gt;::type &gt;</type>
      <name>optional</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>a71e22f3640eb28dd7b2f7b8339931224</anchor>
      <arglist>(T &amp;&amp;value)</arglist>
    </member>
    <member kind="function">
      <type>Optional&lt; T &gt;</type>
      <name>optional</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>a828d28109eb44c02f39b554a392606e9</anchor>
      <arglist>(Args &amp;&amp;... args)</arglist>
    </member>
    <member kind="function">
      <type>auto</type>
      <name>optional</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>ab0ccf001156df9d35c557ce79a66b927</anchor>
      <arglist>(T &amp;&amp;other) -&gt; decltype(Implementation::DeducedOptionalConverter&lt; typename std::decay&lt; T &gt;::type &gt;::from(std::forward&lt; T &gt;(other)))</arglist>
    </member>
    <member kind="variable">
      <type>constexpr NullOptT</type>
      <name>NullOpt</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>a047ff43a43f3841e3c7125e1cc41bb60</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>OptionalStl.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/Containers/</path>
    <filename>OptionalStl_8h</filename>
    <includes id="Optional_8h" name="Optional.h" local="yes" imported="no">Corrade/Containers/Optional.h</includes>
  </compound>
  <compound kind="file">
    <name>Pointer.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/Containers/</path>
    <filename>Pointer_8h</filename>
    <includes id="Tags_8h" name="Tags.h" local="yes" imported="no">Corrade/Containers/Tags.h</includes>
    <includes id="Assert_8h" name="Assert.h" local="yes" imported="no">Corrade/Utility/Assert.h</includes>
    <includes id="Debug_8h" name="Debug.h" local="yes" imported="no">Corrade/Utility/Debug.h</includes>
    <class kind="class">Corrade::Containers::Pointer</class>
    <namespace>Corrade</namespace>
    <namespace>Corrade::Containers</namespace>
    <member kind="function">
      <type>Pointer&lt; T &gt;</type>
      <name>pointer</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>a76b1a48222910763fdc5046b6625090e</anchor>
      <arglist>(T *pointer)</arglist>
    </member>
    <member kind="function">
      <type>auto</type>
      <name>pointer</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>a26b8a841d97121802988aff08bcbb0c6</anchor>
      <arglist>(T &amp;&amp;other) -&gt; decltype(Implementation::DeducedPointerConverter&lt; T &gt;::from(std::move(other)))</arglist>
    </member>
    <member kind="function">
      <type>Pointer&lt; U &gt;</type>
      <name>pointerCast</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>ae1b70d106c3a13b76188c0f7f956cb0c</anchor>
      <arglist>(Pointer&lt; T &gt; &amp;&amp;pointer)</arglist>
    </member>
    <member kind="function">
      <type>Pointer&lt; T &gt;</type>
      <name>pointer</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>ab287dc10fa2591e662d9a9ea611a3dad</anchor>
      <arglist>(Args &amp;&amp;... args)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>PointerStl.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/Containers/</path>
    <filename>PointerStl_8h</filename>
    <includes id="Pointer_8h" name="Pointer.h" local="yes" imported="no">Corrade/Containers/Pointer.h</includes>
  </compound>
  <compound kind="file">
    <name>Reference.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/Containers/</path>
    <filename>Reference_8h</filename>
    <includes id="Debug_8h" name="Debug.h" local="yes" imported="no">Corrade/Utility/Debug.h</includes>
    <class kind="class">Corrade::Containers::Reference</class>
    <namespace>Corrade</namespace>
    <namespace>Corrade::Containers</namespace>
  </compound>
  <compound kind="file">
    <name>ReferenceStl.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/Containers/</path>
    <filename>ReferenceStl_8h</filename>
    <includes id="Reference_8h" name="Reference.h" local="yes" imported="no">Corrade/Containers/Reference.h</includes>
  </compound>
  <compound kind="file">
    <name>ScopedExit.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/Containers/</path>
    <filename>ScopedExit_8h</filename>
    <includes id="ScopeGuard_8h" name="ScopeGuard.h" local="yes" imported="no">Corrade/Containers/ScopeGuard.h</includes>
    <includes id="Macros_8h" name="Macros.h" local="yes" imported="no">Corrade/Utility/Macros.h</includes>
    <namespace>Corrade</namespace>
    <namespace>Corrade::Containers</namespace>
    <member kind="typedef">
      <type>ScopeGuard</type>
      <name>ScopedExit</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>a632d2dde79e960476ba9d9aae7f6f4c6</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>ScopeGuard.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/Containers/</path>
    <filename>ScopeGuard_8h</filename>
    <class kind="class">Corrade::Containers::ScopeGuard</class>
    <namespace>Corrade</namespace>
    <namespace>Corrade::Containers</namespace>
  </compound>
  <compound kind="file">
    <name>StaticArray.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/Containers/</path>
    <filename>StaticArray_8h</filename>
    <includes id="ArrayView_8h" name="ArrayView.h" local="yes" imported="no">Corrade/Containers/ArrayView.h</includes>
    <includes id="Tags_8h" name="Tags.h" local="yes" imported="no">Corrade/Containers/Tags.h</includes>
    <class kind="class">Corrade::Containers::StaticArray</class>
    <namespace>Corrade</namespace>
    <namespace>Corrade::Containers</namespace>
    <member kind="function">
      <type>constexpr ArrayView&lt; T &gt;</type>
      <name>arrayView</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>aa8111199cdb6cc30c470e3cc9c54a30e</anchor>
      <arglist>(StaticArray&lt; size, T &gt; &amp;array)</arglist>
    </member>
    <member kind="function">
      <type>constexpr ArrayView&lt; const T &gt;</type>
      <name>arrayView</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>a5b40a450ade4927381bf97a165e3621e</anchor>
      <arglist>(const StaticArray&lt; size, T &gt; &amp;array)</arglist>
    </member>
    <member kind="function">
      <type>constexpr StaticArrayView&lt; size, T &gt;</type>
      <name>staticArrayView</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>a4f6c86de70a1ab496a1d798648c22f80</anchor>
      <arglist>(StaticArray&lt; size, T &gt; &amp;array)</arglist>
    </member>
    <member kind="function">
      <type>constexpr StaticArrayView&lt; size, const T &gt;</type>
      <name>staticArrayView</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>ab15dc352f75f85e562eaa85ae058ac9f</anchor>
      <arglist>(const StaticArray&lt; size, T &gt; &amp;array)</arglist>
    </member>
    <member kind="function">
      <type>StaticArrayView&lt; size *sizeof(T)/sizeof(U), U &gt;</type>
      <name>arrayCast</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>a5ebf447d7f388e81fe6d34910c80cfbe</anchor>
      <arglist>(StaticArray&lt; size, T &gt; &amp;array)</arglist>
    </member>
    <member kind="function">
      <type>StaticArrayView&lt; size *sizeof(T)/sizeof(U), const U &gt;</type>
      <name>arrayCast</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>aae0b8c37cdca03924fac3a3ff457bf9b</anchor>
      <arglist>(const StaticArray&lt; size, T &gt; &amp;array)</arglist>
    </member>
    <member kind="function">
      <type>constexpr std::size_t</type>
      <name>arraySize</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>a0cad6e09b65666040bb2460df883e4a3</anchor>
      <arglist>(const StaticArray&lt; size_, T &gt; &amp;)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>StridedArrayView.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/Containers/</path>
    <filename>StridedArrayView_8h</filename>
    <includes id="ArrayView_8h" name="ArrayView.h" local="yes" imported="no">Corrade/Containers/ArrayView.h</includes>
    <includes id="Tags_8h" name="Tags.h" local="yes" imported="no">Corrade/Containers/Tags.h</includes>
    <class kind="class">Corrade::Containers::StridedDimensions</class>
    <class kind="class">Corrade::Containers::StridedArrayView</class>
    <class kind="class">Corrade::Containers::StridedIterator</class>
    <namespace>Corrade</namespace>
    <namespace>Corrade::Containers</namespace>
    <member kind="typedef">
      <type>StridedArrayView&lt; 1, T &gt;</type>
      <name>StridedArrayView1D</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>a161bca60e6aa4f6f998b6f3cfa64eaef</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>StridedArrayView&lt; 2, T &gt;</type>
      <name>StridedArrayView2D</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>aad0779dab68e1061b22da995439f668b</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>StridedArrayView&lt; 3, T &gt;</type>
      <name>StridedArrayView3D</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>a8fd4827befab8d70f0da5e4300bfdc21</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>StridedArrayView&lt; 4, T &gt;</type>
      <name>StridedArrayView4D</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>ac01ba7473315887995a8680b259c9c74</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type>constexpr StridedArrayView1D&lt; T &gt;</type>
      <name>stridedArrayView</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>a28f0f3e4514d0a6d4d463f7bd2884f16</anchor>
      <arglist>(T(&amp;data)[size])</arglist>
    </member>
    <member kind="function">
      <type>constexpr StridedArrayView1D&lt; T &gt;</type>
      <name>stridedArrayView</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>a8a18ea77b75e5838af7828f4b681de1d</anchor>
      <arglist>(ArrayView&lt; T &gt; view)</arglist>
    </member>
    <member kind="function">
      <type>constexpr StridedArrayView1D&lt; T &gt;</type>
      <name>stridedArrayView</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>a5687e5c332876211a23a2202ea89231b</anchor>
      <arglist>(StaticArrayView&lt; size, T &gt; view)</arglist>
    </member>
    <member kind="function">
      <type>constexpr StridedArrayView&lt; dimensions, T &gt;</type>
      <name>stridedArrayView</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>ab15f7dff3df81f6529c2b9e401f813dc</anchor>
      <arglist>(StridedArrayView&lt; dimensions, T &gt; view)</arglist>
    </member>
    <member kind="function">
      <type>constexpr U</type>
      <name>stridedArrayView</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>af0b91e302ea8ee6b0c76fe99fe6af1f8</anchor>
      <arglist>(T &amp;&amp;other)</arglist>
    </member>
    <member kind="function">
      <type>StridedArrayView&lt; dimensions, U &gt;</type>
      <name>arrayCast</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>a529bc0bc79d6272d1e53c85c5ecd7005</anchor>
      <arglist>(const StridedArrayView&lt; dimensions, T &gt; &amp;view)</arglist>
    </member>
    <member kind="function">
      <type>StridedArrayView&lt; newDimensions, U &gt;</type>
      <name>arrayCast</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>abb1caf4042587dc306fc92204c441feb</anchor>
      <arglist>(const StridedArrayView&lt; dimensions, T &gt; &amp;view)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>Tags.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/Containers/</path>
    <filename>Tags_8h</filename>
    <class kind="struct">Corrade::Containers::DefaultInitT</class>
    <class kind="struct">Corrade::Containers::ValueInitT</class>
    <class kind="struct">Corrade::Containers::NoInitT</class>
    <class kind="struct">Corrade::Containers::NoCreateT</class>
    <class kind="struct">Corrade::Containers::DirectInitT</class>
    <class kind="struct">Corrade::Containers::InPlaceInitT</class>
    <namespace>Corrade</namespace>
    <namespace>Corrade::Containers</namespace>
    <member kind="variable">
      <type>constexpr DefaultInitT</type>
      <name>DefaultInit</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>a10f8c95aaa0d51bb77976fe3e4924a5e</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>constexpr ValueInitT</type>
      <name>ValueInit</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>a26d49cbb15d20d7dc72878f522e24444</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>constexpr NoInitT</type>
      <name>NoInit</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>a7094104336b357363773b3f29891d048</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>constexpr NoCreateT</type>
      <name>NoCreate</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>a4626e472e29e045795dc8d55909b3e3f</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>constexpr DirectInitT</type>
      <name>DirectInit</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>a69da0a827c69b45cec539ec0c201a119</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>constexpr InPlaceInitT</type>
      <name>InPlaceInit</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>ab86b6e81b5157279ebfbf2b716fb2d6d</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>Corrade.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/</path>
    <filename>Corrade_8h</filename>
    <namespace>Corrade</namespace>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_MSVC2019_COMPATIBILITY</name>
      <anchorfile>Corrade_8h.html</anchorfile>
      <anchor>a430bd5a7eda3b6fb8b35d0938bc34d8f</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_MSVC2017_COMPATIBILITY</name>
      <anchorfile>Corrade_8h.html</anchorfile>
      <anchor>a553f541a45d293e05e7aebb726ed49aa</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_MSVC2015_COMPATIBILITY</name>
      <anchorfile>Corrade_8h.html</anchorfile>
      <anchor>abf7db3f63bdd0deb0fb40a9555e8c56a</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_BUILD_DEPRECATED</name>
      <anchorfile>Corrade_8h.html</anchorfile>
      <anchor>a3768587aab2fe63f56887ce5367594d5</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_BUILD_STATIC</name>
      <anchorfile>Corrade_8h.html</anchorfile>
      <anchor>a18e68fa3008fa4f6f734cc69cb94c133</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_BUILD_MULTITHREADED</name>
      <anchorfile>Corrade_8h.html</anchorfile>
      <anchor>a1ac88f5e5f8c979d585486d09d3f2a76</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_CXX_STANDARD</name>
      <anchorfile>Corrade_8h.html</anchorfile>
      <anchor>ac60c76b712cf20807a6415c733ca40a5</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_TARGET_UNIX</name>
      <anchorfile>Corrade_8h.html</anchorfile>
      <anchor>a89eaf7bef5858f7e783a4e57aec81c6f</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_TARGET_APPLE</name>
      <anchorfile>Corrade_8h.html</anchorfile>
      <anchor>ae2cc32d00e885ea5101b9397ca3bb384</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_TARGET_IOS</name>
      <anchorfile>Corrade_8h.html</anchorfile>
      <anchor>a28ef41f421f7abcd207c8da78fc3c1da</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_TARGET_IOS_SIMULATOR</name>
      <anchorfile>Corrade_8h.html</anchorfile>
      <anchor>abd967ec380b0645c82d7d0cf17106313</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_TARGET_WINDOWS</name>
      <anchorfile>Corrade_8h.html</anchorfile>
      <anchor>a19063435bd2d1b9a6a2567c8c316cb56</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_TARGET_WINDOWS_RT</name>
      <anchorfile>Corrade_8h.html</anchorfile>
      <anchor>aa0551725e55f4584267108f34d07174a</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_TARGET_EMSCRIPTEN</name>
      <anchorfile>Corrade_8h.html</anchorfile>
      <anchor>aafeb56971752236fd87733937f9392f6</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_TARGET_ANDROID</name>
      <anchorfile>Corrade_8h.html</anchorfile>
      <anchor>a4d1ec6acb651e40f04ad8aef103f51a2</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_TARGET_X86</name>
      <anchorfile>Corrade_8h.html</anchorfile>
      <anchor>ae205d17d7e08d468eb9fd141467c3f14</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_TARGET_ARM</name>
      <anchorfile>Corrade_8h.html</anchorfile>
      <anchor>a197c45a30d0cc37b5ffecb9943cdfb9a</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_TARGET_POWERPC</name>
      <anchorfile>Corrade_8h.html</anchorfile>
      <anchor>a1b7f626b036b53c0315f516a8d44df7c</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_TARGET_LIBCXX</name>
      <anchorfile>Corrade_8h.html</anchorfile>
      <anchor>a53878336788ffb39c14bb51d1ca05076</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_TARGET_LIBSTDCXX</name>
      <anchorfile>Corrade_8h.html</anchorfile>
      <anchor>a157b7c44625caeea833bee41adafde7c</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_TARGET_DINKUMWARE</name>
      <anchorfile>Corrade_8h.html</anchorfile>
      <anchor>ad462c1c8f92ad3084169f1ef08ac534f</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_PLUGINMANAGER_NO_DYNAMIC_PLUGIN_SUPPORT</name>
      <anchorfile>Corrade_8h.html</anchorfile>
      <anchor>a17304cb82d4f7b7279a223fe69bb78d8</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_TESTSUITE_TARGET_XCTEST</name>
      <anchorfile>Corrade_8h.html</anchorfile>
      <anchor>ad07f99b330b2e84932d57b2cb7ed393d</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_UTILITY_USE_ANSI_COLORS</name>
      <anchorfile>Corrade_8h.html</anchorfile>
      <anchor>ac7cb3a2eb43e3bc3e9e925899229a768</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>Connection.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/Interconnect/</path>
    <filename>Connection_8h</filename>
    <includes id="Reference_8h" name="Reference.h" local="yes" imported="no">Corrade/Containers/Reference.h</includes>
    <includes id="Interconnect_8h" name="Interconnect.h" local="yes" imported="no">Corrade/Interconnect/Interconnect.h</includes>
    <class kind="class">Corrade::Interconnect::Connection</class>
    <namespace>Corrade</namespace>
    <namespace>Corrade::Interconnect</namespace>
  </compound>
  <compound kind="file">
    <name>Emitter.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/Interconnect/</path>
    <filename>Emitter_8h</filename>
    <includes id="Connection_8h" name="Connection.h" local="yes" imported="no">Corrade/Interconnect/Connection.h</includes>
    <includes id="Assert_8h" name="Assert.h" local="yes" imported="no">Corrade/Utility/Assert.h</includes>
    <class kind="class">Corrade::Interconnect::Emitter</class>
    <class kind="class">Corrade::Interconnect::Emitter::Signal</class>
    <namespace>Corrade</namespace>
    <namespace>Corrade::Interconnect</namespace>
    <member kind="function">
      <type>Connection</type>
      <name>connect</name>
      <anchorfile>namespaceCorrade_1_1Interconnect.html</anchorfile>
      <anchor>afe6ec8e2281aa5a2804094419d135005</anchor>
      <arglist>(EmitterObject &amp;emitter, Interconnect::Emitter::Signal(Emitter::*signal)(Args...), Functor &amp;&amp;slot)</arglist>
    </member>
    <member kind="function">
      <type>Connection</type>
      <name>connect</name>
      <anchorfile>namespaceCorrade_1_1Interconnect.html</anchorfile>
      <anchor>a52f95755ab08bad47be07955f641f2cc</anchor>
      <arglist>(EmitterObject &amp;emitter, Interconnect::Emitter::Signal(Emitter::*signal)(Args...), ReceiverObject &amp;receiver, void(Receiver::*slot)(Args...))</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>disconnect</name>
      <anchorfile>namespaceCorrade_1_1Interconnect.html</anchorfile>
      <anchor>a08518bbf1547ae48fbd32afeba6fc787</anchor>
      <arglist>(Emitter &amp;emitter, const Connection &amp;connection)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>Interconnect.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/Interconnect/</path>
    <filename>Interconnect_8h</filename>
    <class kind="class">Corrade::Interconnect::StateMachine</class>
    <namespace>Corrade</namespace>
    <namespace>Corrade::Interconnect</namespace>
  </compound>
  <compound kind="file">
    <name>Receiver.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/Interconnect/</path>
    <filename>Receiver_8h</filename>
    <includes id="Interconnect_8h" name="Interconnect.h" local="yes" imported="no">Corrade/Interconnect/Interconnect.h</includes>
    <class kind="class">Corrade::Interconnect::Receiver</class>
    <namespace>Corrade</namespace>
    <namespace>Corrade::Interconnect</namespace>
  </compound>
  <compound kind="file">
    <name>StateMachine.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/Interconnect/</path>
    <filename>StateMachine_8h</filename>
    <includes id="Emitter_8h" name="Emitter.h" local="yes" imported="no">Corrade/Interconnect/Emitter.h</includes>
    <class kind="class">Corrade::Interconnect::StateTransition</class>
    <class kind="class">Corrade::Interconnect::StateMachine</class>
    <namespace>Corrade</namespace>
    <namespace>Corrade::Interconnect</namespace>
  </compound>
  <compound kind="file">
    <name>AbstractManager.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/PluginManager/</path>
    <filename>AbstractManager_8h</filename>
    <includes id="EnumSet_8h" name="EnumSet.h" local="yes" imported="no">Corrade/Containers/EnumSet.h</includes>
    <includes id="Pointer_8h" name="Pointer.h" local="yes" imported="no">Corrade/Containers/Pointer.h</includes>
    <includes id="PluginManager_8h" name="PluginManager.h" local="yes" imported="no">Corrade/PluginManager/PluginManager.h</includes>
    <includes id="Resource_8h" name="Resource.h" local="yes" imported="no">Corrade/Utility/Resource.h</includes>
    <includes id="StlForwardString_8h" name="StlForwardString.h" local="yes" imported="no">Corrade/Utility/StlForwardString.h</includes>
    <includes id="StlForwardVector_8h" name="StlForwardVector.h" local="yes" imported="no">Corrade/Utility/StlForwardVector.h</includes>
    <includes id="Utility_8h" name="Utility.h" local="yes" imported="no">Corrade/Utility/Utility.h</includes>
    <class kind="class">Corrade::PluginManager::AbstractManager</class>
    <namespace>Corrade</namespace>
    <namespace>Corrade::PluginManager</namespace>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_PLUGIN_IMPORT</name>
      <anchorfile>AbstractManager_8h.html</anchorfile>
      <anchor>a4ced3bca898533c1f46abcd6ec64342b</anchor>
      <arglist>(name)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_PLUGIN_VERSION</name>
      <anchorfile>AbstractManager_8h.html</anchorfile>
      <anchor>a32a64a27920c39102eec2496320a0b8c</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_PLUGIN_REGISTER</name>
      <anchorfile>AbstractManager_8h.html</anchorfile>
      <anchor>aa39d441432b9f4e399c152c85a6dd417</anchor>
      <arglist>(name, className, interface)</arglist>
    </member>
    <member kind="typedef">
      <type>Containers::EnumSet&lt; LoadState &gt;</type>
      <name>LoadStates</name>
      <anchorfile>namespaceCorrade_1_1PluginManager.html</anchorfile>
      <anchor>a61b14635bc0c147c65e8be80260ca891</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumeration">
      <type></type>
      <name>LoadState</name>
      <anchorfile>namespaceCorrade_1_1PluginManager.html</anchorfile>
      <anchor>aa240e935222a178e1acb5c0853f15547</anchor>
      <arglist></arglist>
      <enumvalue file="namespaceCorrade_1_1PluginManager.html" anchor="aa240e935222a178e1acb5c0853f15547a38c300f4fc9ce8a77aad4a30de05cad8">NotFound</enumvalue>
      <enumvalue file="namespaceCorrade_1_1PluginManager.html" anchor="aa240e935222a178e1acb5c0853f15547a3485912f892918a4ed6b357abad8e5a4">WrongPluginVersion</enumvalue>
      <enumvalue file="namespaceCorrade_1_1PluginManager.html" anchor="aa240e935222a178e1acb5c0853f15547ae90c7458dc20c6326d3a34071de85e3f">WrongInterfaceVersion</enumvalue>
      <enumvalue file="namespaceCorrade_1_1PluginManager.html" anchor="aa240e935222a178e1acb5c0853f15547ad6ab066e75221cc9a69bfede5f75347c">WrongMetadataFile</enumvalue>
      <enumvalue file="namespaceCorrade_1_1PluginManager.html" anchor="aa240e935222a178e1acb5c0853f15547a3c062ee3a4f39445ef4ba44ada5bde3c">UnresolvedDependency</enumvalue>
      <enumvalue file="namespaceCorrade_1_1PluginManager.html" anchor="aa240e935222a178e1acb5c0853f15547a3729d9667c9ed82ae96b6174b288a3a5">LoadFailed</enumvalue>
      <enumvalue file="namespaceCorrade_1_1PluginManager.html" anchor="aa240e935222a178e1acb5c0853f15547a84a8921b25f505d0d2077aeb5db4bc16">Static</enumvalue>
      <enumvalue file="namespaceCorrade_1_1PluginManager.html" anchor="aa240e935222a178e1acb5c0853f15547a7381d487d18845b379422325c0a768d6">Loaded</enumvalue>
      <enumvalue file="namespaceCorrade_1_1PluginManager.html" anchor="aa240e935222a178e1acb5c0853f15547a5111e24c1ecc6266ce0de4b4dc42033b">NotLoaded</enumvalue>
      <enumvalue file="namespaceCorrade_1_1PluginManager.html" anchor="aa240e935222a178e1acb5c0853f15547ac7de13f47f40f470d0795d224221a577">UnloadFailed</enumvalue>
      <enumvalue file="namespaceCorrade_1_1PluginManager.html" anchor="aa240e935222a178e1acb5c0853f15547ab651efdb98a5d6bd2b3935d0c3f4a5e2">Required</enumvalue>
      <enumvalue file="namespaceCorrade_1_1PluginManager.html" anchor="aa240e935222a178e1acb5c0853f15547a019d1ca7d50cc54b995f60d456435e87">Used</enumvalue>
    </member>
    <member kind="function">
      <type>Utility::Debug &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>namespaceCorrade_1_1PluginManager.html</anchorfile>
      <anchor>a73b06a0cd1333798b796a2061dc6bedd</anchor>
      <arglist>(Utility::Debug &amp;debug, PluginManager::LoadState value)</arglist>
    </member>
    <member kind="function">
      <type>Utility::Debug &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>namespaceCorrade_1_1PluginManager.html</anchorfile>
      <anchor>a75f06adcf2b3f416601f1f58507a3f15</anchor>
      <arglist>(Utility::Debug &amp;debug, PluginManager::LoadStates value)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>AbstractManagingPlugin.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/PluginManager/</path>
    <filename>AbstractManagingPlugin_8h</filename>
    <includes id="AbstractPlugin_8h" name="AbstractPlugin.h" local="yes" imported="no">Corrade/PluginManager/AbstractPlugin.h</includes>
    <includes id="Manager_8h" name="Manager.h" local="yes" imported="no">Corrade/PluginManager/Manager.h</includes>
    <class kind="class">Corrade::PluginManager::AbstractManagingPlugin</class>
    <namespace>Corrade</namespace>
    <namespace>Corrade::PluginManager</namespace>
  </compound>
  <compound kind="file">
    <name>AbstractPlugin.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/PluginManager/</path>
    <filename>AbstractPlugin_8h</filename>
    <includes id="Pointer_8h" name="Pointer.h" local="yes" imported="no">Corrade/Containers/Pointer.h</includes>
    <includes id="PluginManager_8h" name="PluginManager.h" local="yes" imported="no">Corrade/PluginManager/PluginManager.h</includes>
    <includes id="StlForwardString_8h" name="StlForwardString.h" local="yes" imported="no">Corrade/Utility/StlForwardString.h</includes>
    <includes id="Utility_8h" name="Utility.h" local="yes" imported="no">Corrade/Utility/Utility.h</includes>
    <includes id="StlForwardVector_8h" name="StlForwardVector.h" local="yes" imported="no">Corrade/Utility/StlForwardVector.h</includes>
    <class kind="class">Corrade::PluginManager::AbstractPlugin</class>
    <namespace>Corrade</namespace>
    <namespace>Corrade::PluginManager</namespace>
  </compound>
  <compound kind="file">
    <name>Manager.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/PluginManager/</path>
    <filename>Manager_8h</filename>
    <includes id="Pointer_8h" name="Pointer.h" local="yes" imported="no">Corrade/Containers/Pointer.h</includes>
    <includes id="AbstractManager_8h" name="AbstractManager.h" local="yes" imported="no">Corrade/PluginManager/AbstractManager.h</includes>
    <includes id="Macros_8h" name="Macros.h" local="yes" imported="no">Corrade/Utility/Macros.h</includes>
    <includes id="PointerStl_8h" name="PointerStl.h" local="yes" imported="no">Corrade/Containers/PointerStl.h</includes>
    <class kind="class">Corrade::PluginManager::Manager</class>
    <namespace>Corrade</namespace>
    <namespace>Corrade::PluginManager</namespace>
  </compound>
  <compound kind="file">
    <name>PluginManager.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/PluginManager/</path>
    <filename>PluginManager_8h</filename>
    <class kind="class">Corrade::PluginManager::AbstractManagingPlugin</class>
    <class kind="class">Corrade::PluginManager::Manager</class>
    <namespace>Corrade</namespace>
    <namespace>Corrade::PluginManager</namespace>
  </compound>
  <compound kind="file">
    <name>PluginMetadata.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/PluginManager/</path>
    <filename>PluginMetadata_8h</filename>
    <includes id="PluginManager_8h" name="PluginManager.h" local="yes" imported="no">Corrade/PluginManager/PluginManager.h</includes>
    <includes id="Utility_8h" name="Utility.h" local="yes" imported="no">Corrade/Utility/Utility.h</includes>
    <class kind="class">Corrade::PluginManager::PluginMetadata</class>
    <namespace>Corrade</namespace>
    <namespace>Corrade::PluginManager</namespace>
  </compound>
  <compound kind="file">
    <name>Comparator.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/TestSuite/</path>
    <filename>Comparator_8h</filename>
    <includes id="Assert_8h" name="Assert.h" local="yes" imported="no">Corrade/Utility/Assert.h</includes>
    <includes id="Debug_8h" name="Debug.h" local="yes" imported="no">Corrade/Utility/Debug.h</includes>
    <includes id="StlForwardString_8h" name="StlForwardString.h" local="yes" imported="no">Corrade/Utility/StlForwardString.h</includes>
    <class kind="class">Corrade::TestSuite::Comparator</class>
    <namespace>Corrade</namespace>
    <namespace>Corrade::TestSuite</namespace>
    <member kind="typedef">
      <type>Containers::EnumSet&lt; ComparisonStatusFlag &gt;</type>
      <name>ComparisonStatusFlags</name>
      <anchorfile>namespaceCorrade_1_1TestSuite.html</anchorfile>
      <anchor>a4578a3861445b1ed4bd30a160f9380bc</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumeration">
      <type></type>
      <name>ComparisonStatusFlag</name>
      <anchorfile>namespaceCorrade_1_1TestSuite.html</anchorfile>
      <anchor>a1dd98653ce2b732a42061f6cb18c7831</anchor>
      <arglist></arglist>
      <enumvalue file="namespaceCorrade_1_1TestSuite.html" anchor="a1dd98653ce2b732a42061f6cb18c7831ad7c8c85bf79bbe1b7188497c32c3b0ca">Failed</enumvalue>
      <enumvalue file="namespaceCorrade_1_1TestSuite.html" anchor="a1dd98653ce2b732a42061f6cb18c7831a0eaadb4fcb48a0a0ed7bc9868be9fbaa">Warning</enumvalue>
      <enumvalue file="namespaceCorrade_1_1TestSuite.html" anchor="a1dd98653ce2b732a42061f6cb18c7831a4c2a8fe7eaf24721cc7a9f0175115bd4">Message</enumvalue>
      <enumvalue file="namespaceCorrade_1_1TestSuite.html" anchor="a1dd98653ce2b732a42061f6cb18c7831ad4a9fa383ab700c5bdd6f31cf7df0faf">Verbose</enumvalue>
      <enumvalue file="namespaceCorrade_1_1TestSuite.html" anchor="a1dd98653ce2b732a42061f6cb18c7831a7f84beab04579bef70043ca0cc72fb85">Diagnostic</enumvalue>
      <enumvalue file="namespaceCorrade_1_1TestSuite.html" anchor="a1dd98653ce2b732a42061f6cb18c7831ad86c142bfd1ab654a16c8eaf191eade7">VerboseDiagnostic</enumvalue>
    </member>
    <member kind="function">
      <type>Utility::Debug &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>namespaceCorrade_1_1TestSuite.html</anchorfile>
      <anchor>a1c1c3434f9c7b2b2902b319c1f87e81c</anchor>
      <arglist>(Utility::Debug &amp;debug, ComparisonStatusFlag value)</arglist>
    </member>
    <member kind="function">
      <type>Utility::Debug &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>namespaceCorrade_1_1TestSuite.html</anchorfile>
      <anchor>a9f9c017468d30a59a39b718d1ff4a91e</anchor>
      <arglist>(Utility::Debug &amp;debug, ComparisonStatusFlags value)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>Container.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/TestSuite/Compare/</path>
    <filename>Container_8h</filename>
    <includes id="Comparator_8h" name="Comparator.h" local="yes" imported="no">Corrade/TestSuite/Comparator.h</includes>
    <class kind="class">Corrade::TestSuite::Compare::Container</class>
    <namespace>Corrade</namespace>
    <namespace>Corrade::TestSuite</namespace>
    <namespace>Corrade::TestSuite::Compare</namespace>
  </compound>
  <compound kind="file">
    <name>File.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/TestSuite/Compare/</path>
    <filename>File_8h</filename>
    <includes id="TestSuite_8h" name="TestSuite.h" local="yes" imported="no">Corrade/TestSuite/TestSuite.h</includes>
    <includes id="Utility_8h" name="Utility.h" local="yes" imported="no">Corrade/Utility/Utility.h</includes>
    <class kind="class">Corrade::TestSuite::Compare::File</class>
    <namespace>Corrade</namespace>
    <namespace>Corrade::TestSuite</namespace>
    <namespace>Corrade::TestSuite::Compare</namespace>
  </compound>
  <compound kind="file">
    <name>FileToString.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/TestSuite/Compare/</path>
    <filename>FileToString_8h</filename>
    <includes id="TestSuite_8h" name="TestSuite.h" local="yes" imported="no">Corrade/TestSuite/TestSuite.h</includes>
    <includes id="Utility_8h" name="Utility.h" local="yes" imported="no">Corrade/Utility/Utility.h</includes>
    <class kind="class">Corrade::TestSuite::Compare::FileToString</class>
    <namespace>Corrade</namespace>
    <namespace>Corrade::TestSuite</namespace>
    <namespace>Corrade::TestSuite::Compare</namespace>
  </compound>
  <compound kind="file">
    <name>FloatingPoint.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/TestSuite/Compare/</path>
    <filename>FloatingPoint_8h</filename>
    <includes id="TestSuite_8h" name="TestSuite.h" local="yes" imported="no">Corrade/TestSuite/TestSuite.h</includes>
    <includes id="Utility_8h" name="Utility.h" local="yes" imported="no">Corrade/Utility/Utility.h</includes>
    <class kind="class">Corrade::TestSuite::Comparator&lt; float &gt;</class>
    <class kind="class">Corrade::TestSuite::Comparator&lt; double &gt;</class>
    <class kind="class">Corrade::TestSuite::Comparator&lt; long double &gt;</class>
    <namespace>Corrade</namespace>
    <namespace>Corrade::TestSuite</namespace>
  </compound>
  <compound kind="file">
    <name>Numeric.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/TestSuite/Compare/</path>
    <filename>Numeric_8h</filename>
    <includes id="Tester_8h" name="Tester.h" local="yes" imported="no">Corrade/TestSuite/Tester.h</includes>
    <includes id="TestSuite_8h" name="TestSuite.h" local="yes" imported="no">Corrade/TestSuite/TestSuite.h</includes>
    <includes id="Debug_8h" name="Debug.h" local="yes" imported="no">Corrade/Utility/Debug.h</includes>
    <class kind="class">Corrade::TestSuite::Compare::Less</class>
    <class kind="class">Corrade::TestSuite::Compare::LessOrEqual</class>
    <class kind="class">Corrade::TestSuite::Compare::GreaterOrEqual</class>
    <class kind="class">Corrade::TestSuite::Compare::Greater</class>
    <class kind="class">Corrade::TestSuite::Compare::Around</class>
    <namespace>Corrade</namespace>
    <namespace>Corrade::TestSuite</namespace>
    <namespace>Corrade::TestSuite::Compare</namespace>
    <member kind="function">
      <type>Around&lt; T &gt;</type>
      <name>around</name>
      <anchorfile>namespaceCorrade_1_1TestSuite_1_1Compare.html</anchorfile>
      <anchor>a76165bdc14d6ddd8ba20356ffd11e2e5</anchor>
      <arglist>(T epsilon)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>SortedContainer.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/TestSuite/Compare/</path>
    <filename>SortedContainer_8h</filename>
    <includes id="Container_8h" name="Container.h" local="yes" imported="no">Corrade/TestSuite/Compare/Container.h</includes>
    <class kind="class">Corrade::TestSuite::Compare::SortedContainer</class>
    <namespace>Corrade</namespace>
    <namespace>Corrade::TestSuite</namespace>
    <namespace>Corrade::TestSuite::Compare</namespace>
  </compound>
  <compound kind="file">
    <name>StringToFile.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/TestSuite/Compare/</path>
    <filename>StringToFile_8h</filename>
    <includes id="TestSuite_8h" name="TestSuite.h" local="yes" imported="no">Corrade/TestSuite/TestSuite.h</includes>
    <includes id="Utility_8h" name="Utility.h" local="yes" imported="no">Corrade/Utility/Utility.h</includes>
    <class kind="class">Corrade::TestSuite::Compare::StringToFile</class>
    <namespace>Corrade</namespace>
    <namespace>Corrade::TestSuite</namespace>
    <namespace>Corrade::TestSuite::Compare</namespace>
  </compound>
  <compound kind="file">
    <name>Tester.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/TestSuite/</path>
    <filename>Tester_8h</filename>
    <includes id="Pointer_8h" name="Pointer.h" local="yes" imported="no">Corrade/Containers/Pointer.h</includes>
    <includes id="Comparator_8h" name="Comparator.h" local="yes" imported="no">Corrade/TestSuite/Comparator.h</includes>
    <includes id="FloatingPoint_8h" name="FloatingPoint.h" local="yes" imported="no">Corrade/TestSuite/Compare/FloatingPoint.h</includes>
    <includes id="Debug_8h" name="Debug.h" local="yes" imported="no">Corrade/Utility/Debug.h</includes>
    <includes id="Macros_8h" name="Macros.h" local="yes" imported="no">Corrade/Utility/Macros.h</includes>
    <includes id="StlForwardString_8h" name="StlForwardString.h" local="yes" imported="no">Corrade/Utility/StlForwardString.h</includes>
    <class kind="class">Corrade::TestSuite::Tester</class>
    <class kind="class">Corrade::TestSuite::Tester::TesterConfiguration</class>
    <namespace>Corrade</namespace>
    <namespace>Corrade::TestSuite</namespace>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_TEST_MAIN</name>
      <anchorfile>Tester_8h.html</anchorfile>
      <anchor>a3c85abc9a7086c802f27679aaa18857d</anchor>
      <arglist>(Class)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_VERIFY</name>
      <anchorfile>Tester_8h.html</anchorfile>
      <anchor>abece56caafaac24db5acc8d2cbcdacf8</anchor>
      <arglist>(expression)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_COMPARE</name>
      <anchorfile>Tester_8h.html</anchorfile>
      <anchor>ac040e8cda2279e803f5cf7ae26fef454</anchor>
      <arglist>(actual, expected)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_COMPARE_AS</name>
      <anchorfile>Tester_8h.html</anchorfile>
      <anchor>a14d3e517a402fb78ccd2535cd02a3ffe</anchor>
      <arglist>(actual, expected, Type...)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_COMPARE_WITH</name>
      <anchorfile>Tester_8h.html</anchorfile>
      <anchor>a100e93eb343f155d19f9af61176c3fd9</anchor>
      <arglist>(actual, expected, comparatorInstance)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_EXPECT_FAIL</name>
      <anchorfile>Tester_8h.html</anchorfile>
      <anchor>abe5c7d7ccc6a5dbb67a4be57ae5ad87f</anchor>
      <arglist>(message)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_EXPECT_FAIL_IF</name>
      <anchorfile>Tester_8h.html</anchorfile>
      <anchor>a541129fcf90cb5371553ad0d056ac2cf</anchor>
      <arglist>(condition, message)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_SKIP</name>
      <anchorfile>Tester_8h.html</anchorfile>
      <anchor>a133aa626c0c87b8c1a600a3a74594c84</anchor>
      <arglist>(message)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_BENCHMARK</name>
      <anchorfile>Tester_8h.html</anchorfile>
      <anchor>accfe2c844655169d9d546734b3d9bc12</anchor>
      <arglist>(batchSize)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>TestSuite.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/TestSuite/</path>
    <filename>TestSuite_8h</filename>
    <includes id="Containers_8h" name="Containers.h" local="yes" imported="no">Corrade/Containers/Containers.h</includes>
    <namespace>Corrade</namespace>
    <namespace>Corrade::TestSuite</namespace>
  </compound>
  <compound kind="file">
    <name>AbstractHash.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/Utility/</path>
    <filename>AbstractHash_8h</filename>
    <includes id="Debug_8h" name="Debug.h" local="yes" imported="no">Corrade/Utility/Debug.h</includes>
    <includes id="DebugStl_8h" name="DebugStl.h" local="yes" imported="no">Corrade/Utility/DebugStl.h</includes>
    <class kind="class">Corrade::Utility::HashDigest</class>
    <class kind="class">Corrade::Utility::AbstractHash</class>
    <namespace>Corrade</namespace>
    <namespace>Corrade::Utility</namespace>
  </compound>
  <compound kind="file">
    <name>AndroidLogStreamBuffer.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/Utility/</path>
    <filename>AndroidLogStreamBuffer_8h</filename>
    <class kind="class">Corrade::Utility::AndroidLogStreamBuffer</class>
    <namespace>Corrade</namespace>
    <namespace>Corrade::Utility</namespace>
  </compound>
  <compound kind="file">
    <name>AndroidStreamBuffer.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/Utility/</path>
    <filename>AndroidStreamBuffer_8h</filename>
    <includes id="Macros_8h" name="Macros.h" local="yes" imported="no">Corrade/Utility/Macros.h</includes>
    <includes id="AndroidLogStreamBuffer_8h" name="AndroidLogStreamBuffer.h" local="yes" imported="no">Corrade/Utility/AndroidLogStreamBuffer.h</includes>
  </compound>
  <compound kind="file">
    <name>Arguments.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/Utility/</path>
    <filename>Arguments_8h</filename>
    <includes id="ConfigurationValue_8h" name="ConfigurationValue.h" local="yes" imported="no">Corrade/Utility/ConfigurationValue.h</includes>
    <includes id="Macros_8h" name="Macros.h" local="yes" imported="no">Corrade/Utility/Macros.h</includes>
    <class kind="class">Corrade::Utility::Arguments</class>
    <namespace>Corrade</namespace>
    <namespace>Corrade::Utility</namespace>
  </compound>
  <compound kind="file">
    <name>Assert.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/Utility/</path>
    <filename>Assert_8h</filename>
    <includes id="Debug_8h" name="Debug.h" local="yes" imported="no">Corrade/Utility/Debug.h</includes>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_NO_ASSERT</name>
      <anchorfile>Assert_8h.html</anchorfile>
      <anchor>a588d67a03f847239f809eda035d73730</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_GRACEFUL_ASSERT</name>
      <anchorfile>Assert_8h.html</anchorfile>
      <anchor>ac1219dd11f8ebd0b77dce3c86b4f1419</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_STANDARD_ASSERT</name>
      <anchorfile>Assert_8h.html</anchorfile>
      <anchor>a845bf87e4589879e6c86c1f49f89c12c</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_ASSERT</name>
      <anchorfile>Assert_8h.html</anchorfile>
      <anchor>a83f7361970111951c88f1564a4f148e8</anchor>
      <arglist>(condition, message, returnValue)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_CONSTEXPR_ASSERT</name>
      <anchorfile>Assert_8h.html</anchorfile>
      <anchor>a43ad70ea0dbc1d959f6e8d1a1ecca845</anchor>
      <arglist>(condition, message)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_ASSERT_OUTPUT</name>
      <anchorfile>Assert_8h.html</anchorfile>
      <anchor>a1e9180e437ee8e687152d1aeb9972cb4</anchor>
      <arglist>(call, message, returnValue)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_INTERNAL_ASSERT</name>
      <anchorfile>Assert_8h.html</anchorfile>
      <anchor>a1114c0fdae83004b9a0f8d2ed4afae96</anchor>
      <arglist>(condition)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_INTERNAL_CONSTEXPR_ASSERT</name>
      <anchorfile>Assert_8h.html</anchorfile>
      <anchor>a9d78e8f103d2bca79e2c3e96b12c2c0c</anchor>
      <arglist>(condition)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_INTERNAL_ASSERT_OUTPUT</name>
      <anchorfile>Assert_8h.html</anchorfile>
      <anchor>a8154b41da944a95b2aab65686a1bc249</anchor>
      <arglist>(call)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_ASSERT_UNREACHABLE</name>
      <anchorfile>Assert_8h.html</anchorfile>
      <anchor>a4cb150db08b529a855e172e3c06e0e71</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>Configuration.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/Utility/</path>
    <filename>Configuration_8h</filename>
    <includes id="EnumSet_8h" name="EnumSet.h" local="yes" imported="no">Corrade/Containers/EnumSet.h</includes>
    <includes id="ConfigurationGroup_8h" name="ConfigurationGroup.h" local="yes" imported="no">Corrade/Utility/ConfigurationGroup.h</includes>
    <class kind="class">Corrade::Utility::Configuration</class>
    <namespace>Corrade</namespace>
    <namespace>Corrade::Utility</namespace>
  </compound>
  <compound kind="file">
    <name>ConfigurationGroup.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/Utility/</path>
    <filename>ConfigurationGroup_8h</filename>
    <includes id="ConfigurationValue_8h" name="ConfigurationValue.h" local="yes" imported="no">Corrade/Utility/ConfigurationValue.h</includes>
    <includes id="Utility_8h" name="Utility.h" local="yes" imported="no">Corrade/Utility/Utility.h</includes>
    <class kind="class">Corrade::Utility::ConfigurationGroup</class>
    <namespace>Corrade</namespace>
    <namespace>Corrade::Utility</namespace>
  </compound>
  <compound kind="file">
    <name>ConfigurationValue.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/Utility/</path>
    <filename>ConfigurationValue_8h</filename>
    <includes id="EnumSet_8h" name="EnumSet.h" local="yes" imported="no">Corrade/Containers/EnumSet.h</includes>
    <includes id="StlForwardString_8h" name="StlForwardString.h" local="yes" imported="no">Corrade/Utility/StlForwardString.h</includes>
    <class kind="struct">Corrade::Utility::ConfigurationValue</class>
    <class kind="struct">Corrade::Utility::ConfigurationValue&lt; short &gt;</class>
    <class kind="struct">Corrade::Utility::ConfigurationValue&lt; unsigned short &gt;</class>
    <class kind="struct">Corrade::Utility::ConfigurationValue&lt; int &gt;</class>
    <class kind="struct">Corrade::Utility::ConfigurationValue&lt; unsigned int &gt;</class>
    <class kind="struct">Corrade::Utility::ConfigurationValue&lt; long &gt;</class>
    <class kind="struct">Corrade::Utility::ConfigurationValue&lt; unsigned long &gt;</class>
    <class kind="struct">Corrade::Utility::ConfigurationValue&lt; long long &gt;</class>
    <class kind="struct">Corrade::Utility::ConfigurationValue&lt; unsigned long long &gt;</class>
    <class kind="struct">Corrade::Utility::ConfigurationValue&lt; float &gt;</class>
    <class kind="struct">Corrade::Utility::ConfigurationValue&lt; double &gt;</class>
    <class kind="struct">Corrade::Utility::ConfigurationValue&lt; long double &gt;</class>
    <class kind="struct">Corrade::Utility::ConfigurationValue&lt; std::string &gt;</class>
    <class kind="struct">Corrade::Utility::ConfigurationValue&lt; bool &gt;</class>
    <class kind="struct">Corrade::Utility::ConfigurationValue&lt; char32_t &gt;</class>
    <namespace>Corrade</namespace>
    <namespace>Corrade::Utility</namespace>
    <member kind="typedef">
      <type>Containers::EnumSet&lt; ConfigurationValueFlag &gt;</type>
      <name>ConfigurationValueFlags</name>
      <anchorfile>namespaceCorrade_1_1Utility.html</anchorfile>
      <anchor>a4d1891e33318b3189e012812127acd8e</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumeration">
      <type></type>
      <name>ConfigurationValueFlag</name>
      <anchorfile>namespaceCorrade_1_1Utility.html</anchorfile>
      <anchor>acac747502f9bde1bf73cbfbe661c780d</anchor>
      <arglist></arglist>
      <enumvalue file="namespaceCorrade_1_1Utility.html" anchor="acac747502f9bde1bf73cbfbe661c780da594be08882c8e9d5efb9eeb62f303744">Oct</enumvalue>
      <enumvalue file="namespaceCorrade_1_1Utility.html" anchor="acac747502f9bde1bf73cbfbe661c780da92640bd72988395b326c888614f8937a">Hex</enumvalue>
      <enumvalue file="namespaceCorrade_1_1Utility.html" anchor="acac747502f9bde1bf73cbfbe661c780da21234a0e100d74037a4da2e53f3200d7">Scientific</enumvalue>
      <enumvalue file="namespaceCorrade_1_1Utility.html" anchor="acac747502f9bde1bf73cbfbe661c780da621e7b8ece62fecc55e883252ff2fbe7">Uppercase</enumvalue>
    </member>
  </compound>
  <compound kind="file">
    <name>Debug.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/Utility/</path>
    <filename>Debug_8h</filename>
    <includes id="Macros_8h" name="Macros.h" local="yes" imported="no">Corrade/Utility/Macros.h</includes>
    <includes id="EnumSet_8h" name="EnumSet.h" local="yes" imported="no">Corrade/Containers/EnumSet.h</includes>
    <includes id="TypeTraits_8h" name="TypeTraits.h" local="yes" imported="no">Corrade/Utility/TypeTraits.h</includes>
    <includes id="Utility_8h" name="Utility.h" local="yes" imported="no">Corrade/Utility/Utility.h</includes>
    <class kind="class">Corrade::Utility::Debug</class>
    <class kind="class">Corrade::Utility::Warning</class>
    <class kind="class">Corrade::Utility::Error</class>
    <class kind="class">Corrade::Utility::Fatal</class>
    <namespace>Corrade</namespace>
    <namespace>Corrade::Utility</namespace>
    <member kind="function">
      <type>Debug &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>namespaceCorrade_1_1Utility.html</anchorfile>
      <anchor>ae2cee4795e84d13d29d1d9e1c805ecd2</anchor>
      <arglist>(Debug &amp;debug, const T &amp;value)</arglist>
    </member>
    <member kind="function">
      <type>Debug &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>namespaceCorrade_1_1Utility.html</anchorfile>
      <anchor>ac103e9217a8710bd03ee4783d3d6ed4b</anchor>
      <arglist>(Debug &amp;debug, const Iterable &amp;value)</arglist>
    </member>
    <member kind="function">
      <type>Debug &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>namespaceCorrade_1_1Utility.html</anchorfile>
      <anchor>a76d39b67ac31b0abf23b5c5d4bec9f15</anchor>
      <arglist>(Debug &amp;debug, const std::pair&lt; T, U &gt; &amp;value)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>DebugStl.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/Utility/</path>
    <filename>DebugStl_8h</filename>
    <includes id="ArrayView_8h" name="ArrayView.h" local="yes" imported="no">Corrade/Containers/ArrayView.h</includes>
    <includes id="Debug_8h" name="Debug.h" local="yes" imported="no">Corrade/Utility/Debug.h</includes>
    <namespace>Corrade</namespace>
    <namespace>Corrade::Utility</namespace>
    <member kind="function">
      <type>Debug &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>namespaceCorrade_1_1Utility.html</anchorfile>
      <anchor>a516b71a8f99c9bd0edea7275889ebc7a</anchor>
      <arglist>(Debug &amp;debug, const std::string &amp;value)</arglist>
    </member>
    <member kind="function">
      <type>Debug &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>namespaceCorrade_1_1Utility.html</anchorfile>
      <anchor>af6d7c9ff8d1c4acaed2c0d5bfe8b4a63</anchor>
      <arglist>(Debug &amp;debug, const std::basic_string&lt; T &gt; &amp;value)</arglist>
    </member>
    <member kind="function">
      <type>Debug &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>namespaceCorrade_1_1Utility.html</anchorfile>
      <anchor>a9f9d27d6f040e6e490f2d73c5a0b8b56</anchor>
      <arglist>(Debug &amp;debug, const std::tuple&lt; Args... &gt; &amp;value)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>Directory.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/Utility/</path>
    <filename>Directory_8h</filename>
    <includes id="Containers_8h" name="Containers.h" local="yes" imported="no">Corrade/Containers/Containers.h</includes>
    <includes id="EnumSet_8h" name="EnumSet.h" local="yes" imported="no">Corrade/Containers/EnumSet.h</includes>
    <includes id="StlForwardString_8h" name="StlForwardString.h" local="yes" imported="no">Corrade/Utility/StlForwardString.h</includes>
    <includes id="StlForwardVector_8h" name="StlForwardVector.h" local="yes" imported="no">Corrade/Utility/StlForwardVector.h</includes>
    <includes id="Macros_8h" name="Macros.h" local="yes" imported="no">Corrade/Utility/Macros.h</includes>
    <namespace>Corrade</namespace>
    <namespace>Corrade::Utility</namespace>
    <namespace>Corrade::Utility::Directory</namespace>
    <member kind="typedef">
      <type>Containers::EnumSet&lt; Flag &gt;</type>
      <name>Flags</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Directory.html</anchorfile>
      <anchor>a9df63241a8c57cdd64316a82bc9713b5</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumeration">
      <type></type>
      <name>Flag</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Directory.html</anchorfile>
      <anchor>a60fc75369f2d2d700d1cc758261cef35</anchor>
      <arglist></arglist>
      <enumvalue file="namespaceCorrade_1_1Utility_1_1Directory.html" anchor="a60fc75369f2d2d700d1cc758261cef35ad6186b962ccb090e812467363c008e0a">SkipDotAndDotDot</enumvalue>
      <enumvalue file="namespaceCorrade_1_1Utility_1_1Directory.html" anchor="a60fc75369f2d2d700d1cc758261cef35ab5fc8c3926425785013c492ab8ecc22f">SkipFiles</enumvalue>
      <enumvalue file="namespaceCorrade_1_1Utility_1_1Directory.html" anchor="a60fc75369f2d2d700d1cc758261cef35a9011abdfaf22b80f4ae98a3f59fed46b">SkipDirectories</enumvalue>
      <enumvalue file="namespaceCorrade_1_1Utility_1_1Directory.html" anchor="a60fc75369f2d2d700d1cc758261cef35a2e58e2c745e14cc46a112c48075b863e">SkipSpecial</enumvalue>
      <enumvalue file="namespaceCorrade_1_1Utility_1_1Directory.html" anchor="a60fc75369f2d2d700d1cc758261cef35a6e1549d1367460b4ab06f5575f2d427c">SortAscending</enumvalue>
      <enumvalue file="namespaceCorrade_1_1Utility_1_1Directory.html" anchor="a60fc75369f2d2d700d1cc758261cef35a12995c419287e0ef1f6f2d6ce1dfc12a">SortDescending</enumvalue>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>fromNativeSeparators</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Directory.html</anchorfile>
      <anchor>a454691f5180e746e5f37d50e5aab9f13</anchor>
      <arglist>(std::string path)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>toNativeSeparators</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Directory.html</anchorfile>
      <anchor>ae912c293390240c7c55cf36a3dd64fc3</anchor>
      <arglist>(std::string path)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>path</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Directory.html</anchorfile>
      <anchor>a222baead7128ec0cadc433e3a8c0059f</anchor>
      <arglist>(const std::string &amp;filename)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>filename</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Directory.html</anchorfile>
      <anchor>a07df998aa551057785f2db6c51e78e63</anchor>
      <arglist>(const std::string &amp;filename)</arglist>
    </member>
    <member kind="function">
      <type>std::pair&lt; std::string, std::string &gt;</type>
      <name>splitExtension</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Directory.html</anchorfile>
      <anchor>a5fd58770313dc7c790213e18dffa3255</anchor>
      <arglist>(const std::string &amp;path)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>join</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Directory.html</anchorfile>
      <anchor>aa1c57ae6d2c15c7507ad1ae70810d4de</anchor>
      <arglist>(const std::string &amp;path, const std::string &amp;filename)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>join</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Directory.html</anchorfile>
      <anchor>a7caa7ac094864b221e993f045827dc29</anchor>
      <arglist>(std::initializer_list&lt; std::string &gt; paths)</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; std::string &gt;</type>
      <name>list</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Directory.html</anchorfile>
      <anchor>a27708f99ad155cdf20dbfb7926478582</anchor>
      <arglist>(const std::string &amp;path, Flags flags=Flags())</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>mkpath</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Directory.html</anchorfile>
      <anchor>ad80859f373fbf1ed39b11eb27649c34b</anchor>
      <arglist>(const std::string &amp;path)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>rm</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Directory.html</anchorfile>
      <anchor>ac78c147e3378045ca84362dc1617a5d2</anchor>
      <arglist>(const std::string &amp;path)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>move</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Directory.html</anchorfile>
      <anchor>aa3505c07378a35988f43dca3496f9631</anchor>
      <arglist>(const std::string &amp;oldPath, const std::string &amp;newPath)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>isSandboxed</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Directory.html</anchorfile>
      <anchor>a3cc7a87483e3b192253cbe72d1cb1f98</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>current</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Directory.html</anchorfile>
      <anchor>ae4f90da841b0501c64f42f062b68c1e6</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>dllLocation</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Directory.html</anchorfile>
      <anchor>a0539be6affb5bd8aa9dee55a47214521</anchor>
      <arglist>(const char *name)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>executableLocation</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Directory.html</anchorfile>
      <anchor>a4e3dbc86c7b8d0c3c39d9802a9947b35</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>home</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Directory.html</anchorfile>
      <anchor>a8dcea7e885d165d77f8ef68814788bd9</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>configurationDir</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Directory.html</anchorfile>
      <anchor>a73ddfdc8249b1bc779e8bb58cd9c319a</anchor>
      <arglist>(const std::string &amp;name)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>tmp</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Directory.html</anchorfile>
      <anchor>a7b3ca5990decccd118261fe86824af70</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>exists</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Directory.html</anchorfile>
      <anchor>a0299a9e5b18b4187615c2fd8e256d586</anchor>
      <arglist>(const std::string &amp;filename)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>fileExists</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Directory.html</anchorfile>
      <anchor>a1a83e17a2230dd6c04d33dac2a6e745f</anchor>
      <arglist>(const std::string &amp;filename)</arglist>
    </member>
    <member kind="function">
      <type>Containers::Array&lt; char &gt;</type>
      <name>read</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Directory.html</anchorfile>
      <anchor>a4a9da22f34b0eb0f409b8183032b46db</anchor>
      <arglist>(const std::string &amp;filename)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>readString</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Directory.html</anchorfile>
      <anchor>afdc9b2319f9470117c4fb6b2ecf63187</anchor>
      <arglist>(const std::string &amp;filename)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>write</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Directory.html</anchorfile>
      <anchor>af3e40dd0b19be0b79e8d2bad631f9009</anchor>
      <arglist>(const std::string &amp;filename, Containers::ArrayView&lt; const void &gt; data)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>writeString</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Directory.html</anchorfile>
      <anchor>a2e31a8897f1f4d78ec7179d584ab0100</anchor>
      <arglist>(const std::string &amp;filename, const std::string &amp;data)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>append</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Directory.html</anchorfile>
      <anchor>aab260374e6387877a6648dfa8696c1b3</anchor>
      <arglist>(const std::string &amp;filename, Containers::ArrayView&lt; const void &gt; data)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>appendString</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Directory.html</anchorfile>
      <anchor>a5dbaa371ae23637c6952fa1360de0f0e</anchor>
      <arglist>(const std::string &amp;filename, const std::string &amp;data)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>copy</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Directory.html</anchorfile>
      <anchor>a665cc812333bddf0c9640376291a8bcb</anchor>
      <arglist>(const std::string &amp;from, const std::string &amp;to)</arglist>
    </member>
    <member kind="function">
      <type>Containers::Array&lt; char, MapDeleter &gt;</type>
      <name>map</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Directory.html</anchorfile>
      <anchor>a769141becd7eed77b15c0d0d1a79cc04</anchor>
      <arglist>(const std::string &amp;filename, std::size_t size)</arglist>
    </member>
    <member kind="function">
      <type>Containers::Array&lt; const char, MapDeleter &gt;</type>
      <name>mapRead</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Directory.html</anchorfile>
      <anchor>a84c45342a8cec2111853f51873d82c9d</anchor>
      <arglist>(const std::string &amp;filename)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>Endianness.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/Utility/</path>
    <filename>Endianness_8h</filename>
    <includes id="utilities_8h" name="utilities.h" local="yes" imported="no">Corrade/Utility/utilities.h</includes>
    <class kind="class">Corrade::Utility::Endianness</class>
    <namespace>Corrade</namespace>
    <namespace>Corrade::Utility</namespace>
  </compound>
  <compound kind="file">
    <name>FileWatcher.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/Utility/</path>
    <filename>FileWatcher_8h</filename>
    <class kind="class">Corrade::Utility::FileWatcher</class>
    <namespace>Corrade</namespace>
    <namespace>Corrade::Utility</namespace>
  </compound>
  <compound kind="file">
    <name>Format.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/Utility/</path>
    <filename>Format_8h</filename>
    <includes id="Containers_8h" name="Containers.h" local="yes" imported="no">Corrade/Containers/Containers.h</includes>
    <namespace>Corrade</namespace>
    <namespace>Corrade::Utility</namespace>
    <member kind="function">
      <type>Containers::Array&lt; char &gt;</type>
      <name>format</name>
      <anchorfile>namespaceCorrade_1_1Utility.html</anchorfile>
      <anchor>a5b85466788fc3d71b143909dda1ef44a</anchor>
      <arglist>(const char *format, const Args &amp;... args)</arglist>
    </member>
    <member kind="function">
      <type>std::size_t</type>
      <name>formatInto</name>
      <anchorfile>namespaceCorrade_1_1Utility.html</anchorfile>
      <anchor>a4cbdd7e7dfea1e5639ee8efa76e78b53</anchor>
      <arglist>(const Containers::ArrayView&lt; char &gt; &amp;buffer, const char *format, const Args &amp;... args)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>formatInto</name>
      <anchorfile>namespaceCorrade_1_1Utility.html</anchorfile>
      <anchor>a3b0e22fcc7bd43f5758df742ba40ebb1</anchor>
      <arglist>(std::FILE *file, const char *format, const Args &amp;... args)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>print</name>
      <anchorfile>namespaceCorrade_1_1Utility.html</anchorfile>
      <anchor>ab27a2c45ff7f820fa421daae6cdb82af</anchor>
      <arglist>(const char *format, const Args &amp;... args)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>printError</name>
      <anchorfile>namespaceCorrade_1_1Utility.html</anchorfile>
      <anchor>a1d88fd05905f0c021fe6169068441102</anchor>
      <arglist>(const char *format, const Args &amp;... args)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>FormatStl.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/Utility/</path>
    <filename>FormatStl_8h</filename>
    <includes id="Format_8h" name="Format.h" local="yes" imported="no">Corrade/Utility/Format.h</includes>
    <namespace>Corrade</namespace>
    <namespace>Corrade::Utility</namespace>
    <member kind="function">
      <type>std::string</type>
      <name>formatString</name>
      <anchorfile>namespaceCorrade_1_1Utility.html</anchorfile>
      <anchor>aaa50b1dc1eaffad13a65ee141a5c9e4f</anchor>
      <arglist>(const char *format, const Args &amp;... args)</arglist>
    </member>
    <member kind="function">
      <type>std::size_t</type>
      <name>formatInto</name>
      <anchorfile>namespaceCorrade_1_1Utility.html</anchorfile>
      <anchor>aaf6e2f081fa8fd67065dd7b39811f5f6</anchor>
      <arglist>(std::string &amp;string, std::size_t offset, const char *format, const Args &amp;... args)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>Macros.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/Utility/</path>
    <filename>Macros_8h</filename>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_DEPRECATED</name>
      <anchorfile>Macros_8h.html</anchorfile>
      <anchor>a8bc844974522ec40054a604cadbdb2b2</anchor>
      <arglist>(message)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_DEPRECATED_ALIAS</name>
      <anchorfile>Macros_8h.html</anchorfile>
      <anchor>a023a57e9d0c61a1b6f840b3e0569f734</anchor>
      <arglist>(message)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_DEPRECATED_NAMESPACE</name>
      <anchorfile>Macros_8h.html</anchorfile>
      <anchor>a0614a11b8608ffc924324f4843da7992</anchor>
      <arglist>(message)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_DEPRECATED_ENUM</name>
      <anchorfile>Macros_8h.html</anchorfile>
      <anchor>aa58b39f18ce825eaad362ea65e91eda1</anchor>
      <arglist>(message)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_DEPRECATED_FILE</name>
      <anchorfile>Macros_8h.html</anchorfile>
      <anchor>a4bc9f2298919bde9bacc16f319707fe3</anchor>
      <arglist>(message)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_DEPRECATED_MACRO</name>
      <anchorfile>Macros_8h.html</anchorfile>
      <anchor>a6a94c59eaa6f104ce689c22ebc44c336</anchor>
      <arglist>(macro, message)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_IGNORE_DEPRECATED_PUSH</name>
      <anchorfile>Macros_8h.html</anchorfile>
      <anchor>a3d9823862d0498c5a6cfe2dbc39f47c5</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_IGNORE_DEPRECATED_POP</name>
      <anchorfile>Macros_8h.html</anchorfile>
      <anchor>a8131d4370d2883c4942e1afebcf18ad5</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_UNUSED</name>
      <anchorfile>Macros_8h.html</anchorfile>
      <anchor>aaf99b187cce2ff1eadddf1507ace2d69</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_ALIGNAS</name>
      <anchorfile>Macros_8h.html</anchorfile>
      <anchor>acb148d614b069112fb97d4f022ab6c9b</anchor>
      <arglist>(alignment)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_NORETURN</name>
      <anchorfile>Macros_8h.html</anchorfile>
      <anchor>aeac429c30ec933b14293d5431a5c1a01</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_THREAD_LOCAL</name>
      <anchorfile>Macros_8h.html</anchorfile>
      <anchor>a59b7cd35ec1f1af9940bd99faed0c7e3</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_ALWAYS_INLINE</name>
      <anchorfile>Macros_8h.html</anchorfile>
      <anchor>a3df4fcf9966b18af3f5d311a04698b56</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_NEVER_INLINE</name>
      <anchorfile>Macros_8h.html</anchorfile>
      <anchor>aa1fa4f001e0054f159136327ede89c6c</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_FUNCTION</name>
      <anchorfile>Macros_8h.html</anchorfile>
      <anchor>a35bb07eaded9ca76d8a2c72135243ded</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_LINE_STRING</name>
      <anchorfile>Macros_8h.html</anchorfile>
      <anchor>a074226a9c37ee5816e828a4cf823381d</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_AUTOMATIC_INITIALIZER</name>
      <anchorfile>Macros_8h.html</anchorfile>
      <anchor>a8a4cea6b4e9cc0f231fec08fc867f2d9</anchor>
      <arglist>(function)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_AUTOMATIC_FINALIZER</name>
      <anchorfile>Macros_8h.html</anchorfile>
      <anchor>a89fc5f07629645f4e45c130859ac0909</anchor>
      <arglist>(function)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>MurmurHash2.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/Utility/</path>
    <filename>MurmurHash2_8h</filename>
    <includes id="AbstractHash_8h" name="AbstractHash.h" local="yes" imported="no">Corrade/Utility/AbstractHash.h</includes>
    <class kind="class">Corrade::Utility::MurmurHash2</class>
    <namespace>Corrade</namespace>
    <namespace>Corrade::Utility</namespace>
  </compound>
  <compound kind="file">
    <name>Resource.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/Utility/</path>
    <filename>Resource_8h</filename>
    <includes id="Containers_8h" name="Containers.h" local="yes" imported="no">Corrade/Containers/Containers.h</includes>
    <includes id="StlForwardString_8h" name="StlForwardString.h" local="yes" imported="no">Corrade/Utility/StlForwardString.h</includes>
    <includes id="StlForwardVector_8h" name="StlForwardVector.h" local="yes" imported="no">Corrade/Utility/StlForwardVector.h</includes>
    <class kind="class">Corrade::Utility::Resource</class>
    <namespace>Corrade</namespace>
    <namespace>Corrade::Utility</namespace>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_RESOURCE_INITIALIZE</name>
      <anchorfile>Resource_8h.html</anchorfile>
      <anchor>a876986c779d3629c74c76e35607ea9e2</anchor>
      <arglist>(name)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_RESOURCE_FINALIZE</name>
      <anchorfile>Resource_8h.html</anchorfile>
      <anchor>a6ded4adbc1776241a0aed878fa5207a9</anchor>
      <arglist>(name)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>Sha1.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/Utility/</path>
    <filename>Sha1_8h</filename>
    <includes id="AbstractHash_8h" name="AbstractHash.h" local="yes" imported="no">Corrade/Utility/AbstractHash.h</includes>
    <class kind="class">Corrade::Utility::Sha1</class>
    <namespace>Corrade</namespace>
    <namespace>Corrade::Utility</namespace>
  </compound>
  <compound kind="file">
    <name>StlForwardArray.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/Utility/</path>
    <filename>StlForwardArray_8h</filename>
  </compound>
  <compound kind="file">
    <name>StlForwardString.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/Utility/</path>
    <filename>StlForwardString_8h</filename>
  </compound>
  <compound kind="file">
    <name>StlForwardTuple.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/Utility/</path>
    <filename>StlForwardTuple_8h</filename>
  </compound>
  <compound kind="file">
    <name>StlForwardVector.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/Utility/</path>
    <filename>StlForwardVector_8h</filename>
  </compound>
  <compound kind="file">
    <name>StlMath.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/Utility/</path>
    <filename>StlMath_8h</filename>
  </compound>
  <compound kind="file">
    <name>String.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/Utility/</path>
    <filename>String_8h</filename>
    <includes id="ArrayView_8h" name="ArrayView.h" local="yes" imported="no">Corrade/Containers/ArrayView.h</includes>
    <namespace>Corrade</namespace>
    <namespace>Corrade::Utility</namespace>
    <namespace>Corrade::Utility::String</namespace>
    <member kind="function">
      <type>std::string</type>
      <name>fromArray</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>a3e5a23b4c3afe46b19c0ea93a098e634</anchor>
      <arglist>(const char *string)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>fromArray</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>acb24339bd65dda380b696d363f4cc799</anchor>
      <arglist>(const char *string, std::size_t length)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>ltrim</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>afc39c7e3c0d3da5bb4cfdf4bada808ab</anchor>
      <arglist>(std::string string, const std::string &amp;characters)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>ltrim</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>a9b120b72f24ac6c1266d60b112a131f6</anchor>
      <arglist>(std::string string, const char(&amp;characters)[size])</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>ltrim</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>a5dcd2f1e07dfeb6b3f523b8fd7fcc0db</anchor>
      <arglist>(std::string string)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>rtrim</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>aa7519775e721a492a57c3752b6e0a5ae</anchor>
      <arglist>(std::string string, const std::string &amp;characters)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>rtrim</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>a787500cae52742d206c30a00d315bd88</anchor>
      <arglist>(std::string string, const char(&amp;characters)[size])</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>rtrim</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>a13384c2eb91136a2282cabcf6d40e7e7</anchor>
      <arglist>(std::string string)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>trim</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>a010626127bd50cc902768b3fa50e8e8c</anchor>
      <arglist>(std::string string, const std::string &amp;characters)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>trim</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>a3e6ed45cb59343ff1c968cc7c5f74864</anchor>
      <arglist>(std::string string, const char(&amp;characters)[size])</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>trim</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>ab1422c3f09858ed4959d7ddd022c913a</anchor>
      <arglist>(std::string string)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>ltrimInPlace</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>a9ec8fbf7b2a097997d9ae0b7fd8008b6</anchor>
      <arglist>(std::string &amp;string, const std::string &amp;characters)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>ltrimInPlace</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>aeb380f7b1a78cde7b825a2dd70d13035</anchor>
      <arglist>(std::string &amp;string, const char(&amp;characters)[size])</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>ltrimInPlace</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>aa559956eafb291ec822be7a090303d93</anchor>
      <arglist>(std::string &amp;string)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rtrimInPlace</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>a7fe6b0a142484d4768e6cbb915229a7e</anchor>
      <arglist>(std::string &amp;string, const std::string &amp;characters)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rtrimInPlace</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>afe0d71afe06d476c99ca801c4a60a13c</anchor>
      <arglist>(std::string &amp;string, const char(&amp;characters)[size])</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rtrimInPlace</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>ae415f2a9274c946aa64ce76b2978f512</anchor>
      <arglist>(std::string &amp;string)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>trimInPlace</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>a60ffdcb3d21f7d4fcf1068be07c37908</anchor>
      <arglist>(std::string &amp;string, const std::string &amp;characters)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>trimInPlace</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>a03e7318d0f0755c771da20b917c294e4</anchor>
      <arglist>(std::string &amp;string, const char(&amp;characters)[size])</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>trimInPlace</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>a9ad609584914e7de5a25fc01d807731b</anchor>
      <arglist>(std::string &amp;string)</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; std::string &gt;</type>
      <name>split</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>ac33538e87b593ea3025b97cb9c4f9c7d</anchor>
      <arglist>(const std::string &amp;string, char delimiter)</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; std::string &gt;</type>
      <name>splitWithoutEmptyParts</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>a223a1c3ccc10a0a1c0d3cea7e1bb8770</anchor>
      <arglist>(const std::string &amp;string, char delimiter)</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; std::string &gt;</type>
      <name>splitWithoutEmptyParts</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>ac0ff6b03a281fecebe8540c3aa86cd47</anchor>
      <arglist>(const std::string &amp;string, const std::string &amp;delimiters)</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; std::string &gt;</type>
      <name>splitWithoutEmptyParts</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>a1360ec5f0e3aa9a34af046480356292f</anchor>
      <arglist>(const std::string &amp;string, const char(&amp;delimiters)[size])</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; std::string &gt;</type>
      <name>splitWithoutEmptyParts</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>ab0ad1369cf991d4a24ad9eb546da5994</anchor>
      <arglist>(const std::string &amp;string)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>join</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>a9af3018407a9cac247c09da9ac8e577d</anchor>
      <arglist>(const std::vector&lt; std::string &gt; &amp;strings, char delimiter)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>joinWithoutEmptyParts</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>ac2adb8eafce8ddb6f7c163bd112e5c57</anchor>
      <arglist>(const std::vector&lt; std::string &gt; &amp;strings, char delimiter)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>lowercase</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>a09fde2ac2c5c7a28e0c694d22ca89ee6</anchor>
      <arglist>(std::string string)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>uppercase</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>a4593d63bc7b1f340584b21d6a169afe9</anchor>
      <arglist>(std::string string)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>beginsWith</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>add091001179ac1455795365a3b667073</anchor>
      <arglist>(const std::string &amp;string, const std::string &amp;prefix)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>beginsWith</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>aca63ac7667c9b71ce5e3240daf32316e</anchor>
      <arglist>(const std::string &amp;string, const char(&amp;prefix)[size])</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>beginsWith</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>adcf054be72915ea30008949ba8d7e45a</anchor>
      <arglist>(const std::string &amp;string, char prefix)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>viewBeginsWith</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>aac2a0377274984a88d8c5ba1267bfaa0</anchor>
      <arglist>(Containers::ArrayView&lt; const char &gt; string, const char(&amp;prefix)[size])</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>viewBeginsWith</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>aba910ca698919200907a1b82b446e0cb</anchor>
      <arglist>(Containers::ArrayView&lt; const char &gt; string, char prefix)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>endsWith</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>a810b5cb07e3a5e5063145ef47567ab7f</anchor>
      <arglist>(const std::string &amp;string, const std::string &amp;suffix)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>endsWith</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>ace94f739418cde7292a1e4ffdb75598e</anchor>
      <arglist>(const std::string &amp;string, const char(&amp;suffix)[size])</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>endsWith</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>a5a89d25c3d54527384d85f3b9b22010f</anchor>
      <arglist>(const std::string &amp;string, char suffix)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>viewEndsWith</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>a036c4916e1cd32b0d31978899c205a9c</anchor>
      <arglist>(Containers::ArrayView&lt; const char &gt; string, const char(&amp;suffix)[size])</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>viewEndsWith</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>a86b3902899551c11f8a1db1b9b0becdf</anchor>
      <arglist>(Containers::ArrayView&lt; const char &gt; string, char suffix)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>stripPrefix</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>a6d1d8723eecd80d2cb0b476a92730782</anchor>
      <arglist>(std::string string, const std::string &amp;prefix)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>stripPrefix</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>a220d3903694aa0a90ec23540054c73cd</anchor>
      <arglist>(std::string string, const char(&amp;prefix)[size])</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>stripPrefix</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>a6dbe4d041b412de515dfaf992f6778b0</anchor>
      <arglist>(std::string string, char prefix)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>stripSuffix</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>a4913f68a09a3ad6dec44e8220d8d4c72</anchor>
      <arglist>(std::string string, const std::string &amp;suffix)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>stripSuffix</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>ab70c0084c70b8dfc512283edb553c5fa</anchor>
      <arglist>(std::string string, const char(&amp;suffix)[size])</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>stripSuffix</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>a376d68bfe85b7c6a6d4a4d87209fcd2b</anchor>
      <arglist>(std::string string, char suffix)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>replaceFirst</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>ae74814f4438913aa1300538529069d32</anchor>
      <arglist>(std::string string, const std::string &amp;search, const std::string &amp;replace)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>replaceFirst</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>a4efbec885c01308ee148007ad21d42ad</anchor>
      <arglist>(std::string string, const char(&amp;search)[searchSize], const char(&amp;replace)[replaceSize])</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>replaceFirst</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>ac7fb659c24110adebb2955a75d290442</anchor>
      <arglist>(std::string string, const char(&amp;search)[searchSize], const std::string &amp;replace)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>replaceFirst</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>a4d54d0ac8fb49d705ae692bc88fa2c85</anchor>
      <arglist>(std::string string, const std::string &amp;search, const char(&amp;replace)[replaceSize])</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>replaceAll</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>aef5cb78b1c8e6336abce1c5b94b6fbf0</anchor>
      <arglist>(std::string string, const std::string &amp;search, const std::string &amp;replace)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>replaceAll</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>a6dd7b108947809be4906cab43e050f0b</anchor>
      <arglist>(std::string string, const char(&amp;search)[searchSize], const char(&amp;replace)[replaceSize])</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>replaceAll</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>a3c0ad6e25f8092de0ac2737ff9c62ee7</anchor>
      <arglist>(std::string string, const char(&amp;search)[searchSize], const std::string &amp;replace)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>replaceAll</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>a47b378ccf712482676488a771ec07f46</anchor>
      <arglist>(std::string string, const std::string &amp;search, const char(&amp;replace)[replaceSize])</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>System.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/Utility/</path>
    <filename>System_8h</filename>
    <namespace>Corrade</namespace>
    <namespace>Corrade::Utility</namespace>
    <namespace>Corrade::Utility::System</namespace>
    <member kind="function">
      <type>void</type>
      <name>sleep</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1System.html</anchorfile>
      <anchor>a7f93179c285d2667b97d9b38476d4cda</anchor>
      <arglist>(std::size_t ms)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>Tweakable.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/Utility/</path>
    <filename>Tweakable_8h</filename>
    <includes id="ArrayView_8h" name="ArrayView.h" local="yes" imported="no">Corrade/Containers/ArrayView.h</includes>
    <includes id="Optional_8h" name="Optional.h" local="yes" imported="no">Corrade/Containers/Optional.h</includes>
    <includes id="Pointer_8h" name="Pointer.h" local="yes" imported="no">Corrade/Containers/Pointer.h</includes>
    <includes id="TweakableParser_8h" name="TweakableParser.h" local="yes" imported="no">Corrade/Utility/TweakableParser.h</includes>
    <includes id="StlForwardString_8h" name="StlForwardString.h" local="yes" imported="no">Corrade/Utility/StlForwardString.h</includes>
    <class kind="class">Corrade::Utility::Tweakable</class>
    <namespace>Corrade</namespace>
    <namespace>Corrade::Utility</namespace>
  </compound>
  <compound kind="file">
    <name>TweakableParser.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/Utility/</path>
    <filename>TweakableParser_8h</filename>
    <includes id="Containers_8h" name="Containers.h" local="yes" imported="no">Corrade/Containers/Containers.h</includes>
    <includes id="Utility_8h" name="Utility.h" local="yes" imported="no">Corrade/Utility/Utility.h</includes>
    <class kind="struct">Corrade::Utility::TweakableParser</class>
    <class kind="struct">Corrade::Utility::TweakableParser&lt; int &gt;</class>
    <class kind="struct">Corrade::Utility::TweakableParser&lt; unsigned int &gt;</class>
    <class kind="struct">Corrade::Utility::TweakableParser&lt; long &gt;</class>
    <class kind="struct">Corrade::Utility::TweakableParser&lt; unsigned long &gt;</class>
    <class kind="struct">Corrade::Utility::TweakableParser&lt; long long &gt;</class>
    <class kind="struct">Corrade::Utility::TweakableParser&lt; unsigned long long &gt;</class>
    <class kind="struct">Corrade::Utility::TweakableParser&lt; float &gt;</class>
    <class kind="struct">Corrade::Utility::TweakableParser&lt; double &gt;</class>
    <class kind="struct">Corrade::Utility::TweakableParser&lt; long double &gt;</class>
    <class kind="struct">Corrade::Utility::TweakableParser&lt; char &gt;</class>
    <class kind="struct">Corrade::Utility::TweakableParser&lt; bool &gt;</class>
    <namespace>Corrade</namespace>
    <namespace>Corrade::Utility</namespace>
    <member kind="enumeration">
      <type></type>
      <name>TweakableState</name>
      <anchorfile>namespaceCorrade_1_1Utility.html</anchorfile>
      <anchor>aa187573406978367665d0642ec5b6b04</anchor>
      <arglist></arglist>
      <enumvalue file="namespaceCorrade_1_1Utility.html" anchor="aa187573406978367665d0642ec5b6b04a4bac8cdf0a968472b519b3b295d0d48b">NoChange</enumvalue>
      <enumvalue file="namespaceCorrade_1_1Utility.html" anchor="aa187573406978367665d0642ec5b6b04a505a83f220c02df2f85c3810cd9ceb38">Success</enumvalue>
      <enumvalue file="namespaceCorrade_1_1Utility.html" anchor="aa187573406978367665d0642ec5b6b04a780b2be805c1437cd6730ba91f6107e3">Recompile</enumvalue>
      <enumvalue file="namespaceCorrade_1_1Utility.html" anchor="aa187573406978367665d0642ec5b6b04a902b0d55fddef6f8d651fe1035b7d4bd">Error</enumvalue>
    </member>
    <member kind="function">
      <type>Debug &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>namespaceCorrade_1_1Utility.html</anchorfile>
      <anchor>ae0d97ad312a2030f1344b8839f5d9779</anchor>
      <arglist>(Debug &amp;debug, TweakableState value)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>TypeTraits.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/Utility/</path>
    <filename>TypeTraits_8h</filename>
    <namespace>Corrade</namespace>
    <namespace>Corrade::Utility</namespace>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_HAS_TYPE</name>
      <anchorfile>TypeTraits_8h.html</anchorfile>
      <anchor>a19ce628bb598766db0fffb24c19fc9de</anchor>
      <arglist>(className, typeExpression)</arglist>
    </member>
    <member kind="typedef">
      <type>std::integral_constant&lt; bool, implementation-specific &gt;</type>
      <name>IsIterable</name>
      <anchorfile>namespaceCorrade_1_1Utility.html</anchorfile>
      <anchor>a5b6f2707e54c886af00f4738ff83afa2</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::integral_constant&lt; bool, implementation-specific &gt;</type>
      <name>IsStringLike</name>
      <anchorfile>namespaceCorrade_1_1Utility.html</anchorfile>
      <anchor>a25706236e8711798719a814357f6bffd</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>Unicode.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/Utility/</path>
    <filename>Unicode_8h</filename>
    <includes id="ArrayView_8h" name="ArrayView.h" local="yes" imported="no">Corrade/Containers/ArrayView.h</includes>
    <includes id="StlForwardString_8h" name="StlForwardString.h" local="yes" imported="no">Corrade/Utility/StlForwardString.h</includes>
    <namespace>Corrade</namespace>
    <namespace>Corrade::Utility</namespace>
    <namespace>Corrade::Utility::Unicode</namespace>
    <member kind="function">
      <type>std::pair&lt; char32_t, std::size_t &gt;</type>
      <name>nextChar</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Unicode.html</anchorfile>
      <anchor>a23fbffa5a6184ab6c29030e152182c29</anchor>
      <arglist>(Containers::ArrayView&lt; const char &gt; text, std::size_t cursor)</arglist>
    </member>
    <member kind="function">
      <type>std::pair&lt; char32_t, std::size_t &gt;</type>
      <name>nextChar</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Unicode.html</anchorfile>
      <anchor>a7689b3d817c5244f7300a8503441865a</anchor>
      <arglist>(const std::string &amp;text, const std::size_t cursor)</arglist>
    </member>
    <member kind="function">
      <type>std::pair&lt; char32_t, std::size_t &gt;</type>
      <name>nextChar</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Unicode.html</anchorfile>
      <anchor>a634c36aab0be25e8417905c1a1b492c4</anchor>
      <arglist>(const char(&amp;text)[size], const std::size_t cursor)</arglist>
    </member>
    <member kind="function">
      <type>std::pair&lt; char32_t, std::size_t &gt;</type>
      <name>prevChar</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Unicode.html</anchorfile>
      <anchor>ab5eed1f9f89a2e19eaef0c0f64fb9900</anchor>
      <arglist>(Containers::ArrayView&lt; const char &gt; text, std::size_t cursor)</arglist>
    </member>
    <member kind="function">
      <type>std::pair&lt; char32_t, std::size_t &gt;</type>
      <name>prevChar</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Unicode.html</anchorfile>
      <anchor>a73e6d76d0393312eece30aa83a5fdd73</anchor>
      <arglist>(const std::string &amp;text, const std::size_t cursor)</arglist>
    </member>
    <member kind="function">
      <type>std::pair&lt; char32_t, std::size_t &gt;</type>
      <name>prevChar</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Unicode.html</anchorfile>
      <anchor>ab7b8f89831f52d67d311c5886b847ce6</anchor>
      <arglist>(const char(&amp;text)[size], const std::size_t cursor)</arglist>
    </member>
    <member kind="function">
      <type>std::u32string</type>
      <name>utf32</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Unicode.html</anchorfile>
      <anchor>a738435f73e123822a70f692d9d9cfec8</anchor>
      <arglist>(const std::string &amp;text)</arglist>
    </member>
    <member kind="function">
      <type>std::size_t</type>
      <name>utf8</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Unicode.html</anchorfile>
      <anchor>a0b1547a3b650117161826a75943e5946</anchor>
      <arglist>(char32_t character, Containers::StaticArrayView&lt; 4, char &gt; result)</arglist>
    </member>
    <member kind="function">
      <type>std::wstring</type>
      <name>widen</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Unicode.html</anchorfile>
      <anchor>af92cf003d4c9bf36b0a53777ab5a0445</anchor>
      <arglist>(const std::string &amp;text)</arglist>
    </member>
    <member kind="function">
      <type>std::wstring</type>
      <name>widen</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Unicode.html</anchorfile>
      <anchor>aa4f142c6268dbd30d3b816e36e9ab37b</anchor>
      <arglist>(Containers::ArrayView&lt; const char &gt; text)</arglist>
    </member>
    <member kind="function">
      <type>std::wstring</type>
      <name>widen</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Unicode.html</anchorfile>
      <anchor>a025f6762f095b8180ce0569573b82380</anchor>
      <arglist>(const char *text)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>narrow</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Unicode.html</anchorfile>
      <anchor>ad81ae601a86426189543698218fb276d</anchor>
      <arglist>(const std::wstring &amp;text)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>narrow</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Unicode.html</anchorfile>
      <anchor>a68b980fd21405108c7eb80d1b07408e6</anchor>
      <arglist>(Containers::ArrayView&lt; const wchar_t &gt; text)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>narrow</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Unicode.html</anchorfile>
      <anchor>a7dfc7f84250f3a9a16f4a8fb4bc7a97b</anchor>
      <arglist>(const wchar_t *text)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>utilities.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/Utility/</path>
    <filename>utilities_8h</filename>
    <namespace>Corrade</namespace>
    <namespace>Corrade::Utility</namespace>
    <member kind="function">
      <type>To</type>
      <name>bitCast</name>
      <anchorfile>namespaceCorrade_1_1Utility.html</anchorfile>
      <anchor>a3831cece822dfde455336664676f6867</anchor>
      <arglist>(const From &amp;from)</arglist>
    </member>
    <member kind="function">
      <type>To</type>
      <name>bitCast</name>
      <anchorfile>namespaceCorrade_1_1Utility.html</anchorfile>
      <anchor>a3831cece822dfde455336664676f6867</anchor>
      <arglist>(const From &amp;from)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>Utility.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/Utility/</path>
    <filename>Utility_8h</filename>
    <includes id="Containers_8h" name="Containers.h" local="yes" imported="no">Corrade/Containers/Containers.h</includes>
    <class kind="class">Corrade::Utility::HashDigest</class>
    <class kind="struct">Corrade::Utility::ConfigurationValue</class>
    <class kind="struct">Corrade::Utility::TweakableParser</class>
    <namespace>Corrade</namespace>
    <namespace>Corrade::Utility</namespace>
  </compound>
  <compound kind="file">
    <name>VisibilityMacros.h</name>
    <path>/home/mosra/Code/corrade/src/Corrade/Utility/</path>
    <filename>VisibilityMacros_8h</filename>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_VISIBILITY_EXPORT</name>
      <anchorfile>VisibilityMacros_8h.html</anchorfile>
      <anchor>a94f5b366975d677886021cd41bfaacfb</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_VISIBILITY_INLINE_MEMBER_EXPORT</name>
      <anchorfile>VisibilityMacros_8h.html</anchorfile>
      <anchor>ac332c9c9f3b148b132e7b52a162fe31e</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_VISIBILITY_IMPORT</name>
      <anchorfile>VisibilityMacros_8h.html</anchorfile>
      <anchor>a62c8c1098f4007c1769d03181a78b33c</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_VISIBILITY_STATIC</name>
      <anchorfile>VisibilityMacros_8h.html</anchorfile>
      <anchor>ac12ca04badf8b2b6bc283e5f0da3802e</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_VISIBILITY_LOCAL</name>
      <anchorfile>VisibilityMacros_8h.html</anchorfile>
      <anchor>a802c9a3721942bf48b11ca6314e14a4a</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>AbstractHash&lt; 20 &gt;</name>
    <filename>classCorrade_1_1Utility_1_1AbstractHash.html</filename>
    <member kind="typedef">
      <type>HashDigest&lt; digestSize &gt;</type>
      <name>Digest</name>
      <anchorfile>classCorrade_1_1Utility_1_1AbstractHash.html</anchorfile>
      <anchor>a44db12441a55b94fdead99c880fd0a4e</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DigestSize</name>
      <anchorfile>classCorrade_1_1Utility_1_1AbstractHash.html</anchorfile>
      <anchor>a44bc30cd5b5469847d54604642376d08a1845bcbe98f41a624c92ed7e797a14f6</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>AbstractHash&lt; sizeof(std::size_t)&gt;</name>
    <filename>classCorrade_1_1Utility_1_1AbstractHash.html</filename>
    <member kind="typedef">
      <type>HashDigest&lt; digestSize &gt;</type>
      <name>Digest</name>
      <anchorfile>classCorrade_1_1Utility_1_1AbstractHash.html</anchorfile>
      <anchor>a44db12441a55b94fdead99c880fd0a4e</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DigestSize</name>
      <anchorfile>classCorrade_1_1Utility_1_1AbstractHash.html</anchorfile>
      <anchor>a44bc30cd5b5469847d54604642376d08a1845bcbe98f41a624c92ed7e797a14f6</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>Array&lt; char &gt;</name>
    <filename>classCorrade_1_1Containers_1_1Array.html</filename>
    <member kind="typedef">
      <type>char</type>
      <name>Type</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>ab88bec3b842462e9eea5157a4049a094</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>void(*)(char *, std::size_t)</type>
      <name>Deleter</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>a6397e3aabed2ca448871a747e19a7ee7</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Array</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>ae5ab628ed1d689df54ee1e14c9f33e00</anchor>
      <arglist>(std::nullptr_t) noexcept</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Array</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>ab4c2451e1504971dee5076ad43300be8</anchor>
      <arglist>() noexcept</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Array</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>a4fcbcb24d5a462f13e7dc8e138664e31</anchor>
      <arglist>(DefaultInitT, std::size_t size)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Array</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>ada8220cec2457b9e1730b71d24e3e60e</anchor>
      <arglist>(ValueInitT, std::size_t size)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Array</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>ad6a7e145daae045d737003b626cc958d</anchor>
      <arglist>(NoInitT, std::size_t size)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Array</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>a61202d7c6f6b313d44c6c3f3f3b2331e</anchor>
      <arglist>(DirectInitT, std::size_t size, Args &amp;&amp;... args)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Array</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>aa48105e1a6a95c402c274d32d8d213c1</anchor>
      <arglist>(InPlaceInitT, std::initializer_list&lt; char &gt; list)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Array</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>a913499c6a05cd36964ad56d0eaa1f002</anchor>
      <arglist>(std::size_t size)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Array</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>ab527926a8480885ca15d7fd53c4e0cef</anchor>
      <arglist>(char *data, std::size_t size, void(*)(char *, std::size_t) deleter=Implementation::DefaultDeleter&lt; void(*)(char *, std::size_t) &gt;{}())</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Array</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>a96d5213dcca0193ecd6c8d67217efde2</anchor>
      <arglist>(const Array&lt; char, void(*)(char *, std::size_t) &gt; &amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Array</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>ade7103081fcb82b77f40b3a6a0c9bd1f</anchor>
      <arglist>(Array&lt; char, void(*)(char *, std::size_t) &gt; &amp;&amp;other) noexcept</arglist>
    </member>
    <member kind="function">
      <type>Array&lt; char, void(*)(char *, std::size_t) &gt; &amp;</type>
      <name>operator=</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>ae4fd95d4d6fb93e26fe27daf744fe1f3</anchor>
      <arglist>(const Array&lt; char, void(*)(char *, std::size_t) &gt; &amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type>Array&lt; char, void(*)(char *, std::size_t) &gt; &amp;</type>
      <name>operator=</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>a41292530583857fa7d7ccfdce00fb3ba</anchor>
      <arglist>(Array&lt; char, void(*)(char *, std::size_t) &gt; &amp;&amp;) noexcept</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>operator U</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>af31e57e0d1e4f2e5d93499c757c8b6df</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>operator U</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>aefbd190121eea9f4c4d3ef05ed9f93d6</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>operator bool</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>a2f5760d29e69b7cd6ed9677d7437dc55</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>operator char *</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>ab0bf774c244d0d7e45a0d6451d27639f</anchor>
      <arglist>() &amp;</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>operator const char *</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>acc671fef58ee49ca4771122488f0802f</anchor>
      <arglist>() const &amp;</arglist>
    </member>
    <member kind="function">
      <type>char *</type>
      <name>data</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>a0c46890d99721a859619561f719b4e31</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const char *</type>
      <name>data</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>a33fe38b5e0d633ff71d09ac0680fdb45</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>void(*)(char *, std::size_t)</type>
      <name>deleter</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>acb21f4e838440faf1fb27c734faafb41</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>std::size_t</type>
      <name>size</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>af1e4ac498e1951951729ecab48134a34</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>empty</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>ad1cee21a514b977d045ae8886f8f8b75</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>char *</type>
      <name>begin</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>abaad6108087008802cd894a6679e2b94</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const char *</type>
      <name>begin</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>a2cf50e3d4736428b83c245135a961044</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>const char *</type>
      <name>cbegin</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>a51dfe4f115f8ed72485d544ab64cf769</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>char *</type>
      <name>end</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>a339172cd6f805a667dfbebdcefa533b9</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const char *</type>
      <name>end</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>a123d78064aa332f26b476a45ef0e36f4</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>const char *</type>
      <name>cend</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>a1533ba643c00b3fea2077fbeb39b2afb</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>char &amp;</type>
      <name>front</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>af9952bb49bfc14336f8cf74a36972df8</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const char &amp;</type>
      <name>front</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>a4f01b9e40d5c4da7e6c1f9f73975c53b</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>char &amp;</type>
      <name>back</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>a09753cc0bf9113ccc38af90b6046885b</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const char &amp;</type>
      <name>back</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>a94b5206d0207696751a14e3a4cdae0f7</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>ArrayView&lt; char &gt;</type>
      <name>slice</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>a31ed9632bbdf1612906867854df485bd</anchor>
      <arglist>(char *begin, char *end)</arglist>
    </member>
    <member kind="function">
      <type>ArrayView&lt; const char &gt;</type>
      <name>slice</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>a9ca17fbe62f246a0ecddf34c2f7bdda6</anchor>
      <arglist>(const char *begin, const char *end) const</arglist>
    </member>
    <member kind="function">
      <type>ArrayView&lt; char &gt;</type>
      <name>slice</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>aa73a1d7ad0ac1447d8d88bf4528fad53</anchor>
      <arglist>(std::size_t begin, std::size_t end)</arglist>
    </member>
    <member kind="function">
      <type>ArrayView&lt; const char &gt;</type>
      <name>slice</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>aa8233e6899d80bdc8b5a29b5d6c06a01</anchor>
      <arglist>(std::size_t begin, std::size_t end) const</arglist>
    </member>
    <member kind="function">
      <type>StaticArrayView&lt; size, char &gt;</type>
      <name>slice</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>a824ad0c5ac1722ca966e69177c136622</anchor>
      <arglist>(char *begin)</arglist>
    </member>
    <member kind="function">
      <type>StaticArrayView&lt; size, const char &gt;</type>
      <name>slice</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>a6601fd7167cd944b0d2a04c5964c7746</anchor>
      <arglist>(const char *begin) const</arglist>
    </member>
    <member kind="function">
      <type>StaticArrayView&lt; size, char &gt;</type>
      <name>slice</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>a3364f1140c218b837780b10b970b326f</anchor>
      <arglist>(std::size_t begin)</arglist>
    </member>
    <member kind="function">
      <type>StaticArrayView&lt; size, const char &gt;</type>
      <name>slice</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>a4a00983e653ca797f8dcbf1cc209ae15</anchor>
      <arglist>(std::size_t begin) const</arglist>
    </member>
    <member kind="function">
      <type>StaticArrayView&lt; end_ - begin_, char &gt;</type>
      <name>slice</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>a4efc7e35b1c6e07f6a0538e70ae74f48</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>StaticArrayView&lt; end_ - begin_, const char &gt;</type>
      <name>slice</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>a7a7d4aff6c7407560052e6a991ce4270</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>ArrayView&lt; char &gt;</type>
      <name>prefix</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>a24be60e39728f7d05ba83b0a8eb506bf</anchor>
      <arglist>(char *end)</arglist>
    </member>
    <member kind="function">
      <type>ArrayView&lt; const char &gt;</type>
      <name>prefix</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>a46534a6cb1a2b3fccad4461b36e16f23</anchor>
      <arglist>(const char *end) const</arglist>
    </member>
    <member kind="function">
      <type>ArrayView&lt; char &gt;</type>
      <name>prefix</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>af67d0fcf514203404db65d818d718208</anchor>
      <arglist>(std::size_t end)</arglist>
    </member>
    <member kind="function">
      <type>ArrayView&lt; const char &gt;</type>
      <name>prefix</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>a72500d94e860cbb4a4f1d74e296529e1</anchor>
      <arglist>(std::size_t end) const</arglist>
    </member>
    <member kind="function">
      <type>StaticArrayView&lt; viewSize, char &gt;</type>
      <name>prefix</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>a09dd6511186942d0a86f932275c932c0</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>StaticArrayView&lt; viewSize, const char &gt;</type>
      <name>prefix</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>a9cd5951dd48d0208a0f2059fd323f62b</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>ArrayView&lt; char &gt;</type>
      <name>suffix</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>a803162c8189d5ce30496dd61d515036b</anchor>
      <arglist>(char *begin)</arglist>
    </member>
    <member kind="function">
      <type>ArrayView&lt; const char &gt;</type>
      <name>suffix</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>a71c394e0726949e796704f63d62a4a86</anchor>
      <arglist>(const char *begin) const</arglist>
    </member>
    <member kind="function">
      <type>ArrayView&lt; char &gt;</type>
      <name>suffix</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>adf6635159fa9c6d4c47c4f98ef0248af</anchor>
      <arglist>(std::size_t begin)</arglist>
    </member>
    <member kind="function">
      <type>ArrayView&lt; const char &gt;</type>
      <name>suffix</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>aad441eb3bb38a897fc974dbd95440609</anchor>
      <arglist>(std::size_t begin) const</arglist>
    </member>
    <member kind="function">
      <type>ArrayView&lt; char &gt;</type>
      <name>except</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>a23d4bc894026561c0aea1ec22d5f6350</anchor>
      <arglist>(std::size_t count)</arglist>
    </member>
    <member kind="function">
      <type>ArrayView&lt; const char &gt;</type>
      <name>except</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>a300375138e0e41448e54cedd9982a485</anchor>
      <arglist>(std::size_t count) const</arglist>
    </member>
    <member kind="function">
      <type>char *</type>
      <name>release</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>aa6256e53419003ba6f434a91f293c92e</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>ArrayView&lt; U &gt;</type>
      <name>arrayCast</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>ad5f0615634b2553b0cb304aa58da3347</anchor>
      <arglist>(Array&lt; char, void(*)(char *, std::size_t) &gt; &amp;array)</arglist>
    </member>
    <member kind="function">
      <type>std::size_t</type>
      <name>arraySize</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>af8a4e6b17dca34a5eb82b6a8da8c398b</anchor>
      <arglist>(const Array&lt; char &gt; &amp;view)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>ArrayView&lt; const char &gt;</name>
    <filename>classCorrade_1_1Containers_1_1ArrayView.html</filename>
    <member kind="typedef">
      <type>const char</type>
      <name>Type</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>acffca460a3bea7a4721e765f069587b2</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>ArrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>a832bfa9a5d0aa6791d995a80e60621b6</anchor>
      <arglist>(std::nullptr_t) noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>ArrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>aa5b6bedc075e0698b30846770d50d312</anchor>
      <arglist>() noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>ArrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>a67f28104cc25005a20a4e1501f264336</anchor>
      <arglist>(const char *data, std::size_t size) noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>ArrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>a4318ae04782dfafbdbbe764bc3e5f15b</anchor>
      <arglist>(U(&amp;data)[size]) noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>ArrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>a983523ac9454f971954d89363bf044c8</anchor>
      <arglist>(ArrayView&lt; U &gt; view) noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>ArrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>a8c3a0e9e3639eb47f6c99b9d72556bf2</anchor>
      <arglist>(StaticArrayView&lt; size, U &gt; view) noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>ArrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>af9b5913220fc5e7a42c5fa3b7ad561a5</anchor>
      <arglist>(U &amp;&amp;other) noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>operator U</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>a230261d1b1f375d09aa74c5f308a09a2</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>operator bool</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>adf0edfab81a28ba75f8d0baee9116310</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>operator const char *</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>af6fb5ae89b589cc272297a04fb587e38</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr const char *</type>
      <name>data</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>ad94873c712a5e8e46698936fb8f47b71</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr std::size_t</type>
      <name>size</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>a878a1f4c48b49f36fbc4922951d60b5c</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr bool</type>
      <name>empty</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>a6759e541efd29f1bd590240aede6c9f7</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr const char *</type>
      <name>begin</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>a0b8ab5ab09f103c21276d14618a5d6bc</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr const char *</type>
      <name>cbegin</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>acb1197d09b279ded50ce6187e04d0c31</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr const char *</type>
      <name>end</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>ae5982c4c3b93bf0ea82544b772bdf460</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr const char *</type>
      <name>cend</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>a13ec5077592c88962836e169226160db</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>const char &amp;</type>
      <name>front</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>a6ffb1b09a34e7f019f1794c75c26c3a1</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>const char &amp;</type>
      <name>back</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>a160b0f8ab187867806627e00d69b8395</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr ArrayView&lt; const char &gt;</type>
      <name>slice</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>aa0af13696fd7aa2d0199eea76789e809</anchor>
      <arglist>(const char *begin, const char *end) const</arglist>
    </member>
    <member kind="function">
      <type>constexpr ArrayView&lt; const char &gt;</type>
      <name>slice</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>addc4457d3b4ad1457e7bf3b89fd63850</anchor>
      <arglist>(std::size_t begin, std::size_t end) const</arglist>
    </member>
    <member kind="function">
      <type>constexpr StaticArrayView&lt; viewSize, const char &gt;</type>
      <name>slice</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>a318f61fb34544495328ea2f796a709b8</anchor>
      <arglist>(const char *begin) const</arglist>
    </member>
    <member kind="function">
      <type>constexpr StaticArrayView&lt; viewSize, const char &gt;</type>
      <name>slice</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>a0bda35382b3f242ea7807cbf6f4546c6</anchor>
      <arglist>(std::size_t begin) const</arglist>
    </member>
    <member kind="function">
      <type>constexpr StaticArrayView&lt; end_ - begin_, const char &gt;</type>
      <name>slice</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>a4d991bb99fef30b7cfa8c027c8c0385e</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr ArrayView&lt; const char &gt;</type>
      <name>prefix</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>a90dac489ab8d3b34aeb4a4073ff7471e</anchor>
      <arglist>(const char *end) const</arglist>
    </member>
    <member kind="function">
      <type>constexpr ArrayView&lt; const char &gt;</type>
      <name>prefix</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>a454aa80cc409514620156ea03c3e1796</anchor>
      <arglist>(std::size_t end) const</arglist>
    </member>
    <member kind="function">
      <type>constexpr StaticArrayView&lt; end_, const char &gt;</type>
      <name>prefix</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>a4f9b26a22db78cfaa22111159c626165</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr ArrayView&lt; const char &gt;</type>
      <name>suffix</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>a259cb9a3a4cc1ba4d68cd92a3debcb4b</anchor>
      <arglist>(const char *begin) const</arglist>
    </member>
    <member kind="function">
      <type>constexpr ArrayView&lt; const char &gt;</type>
      <name>suffix</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>a7afdfccf156f9471b9282c38fee2b5b6</anchor>
      <arglist>(std::size_t begin) const</arglist>
    </member>
    <member kind="function">
      <type>constexpr ArrayView&lt; const char &gt;</type>
      <name>except</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>a0f293b9e5484a13e7b3a36911d2b97da</anchor>
      <arglist>(std::size_t count) const</arglist>
    </member>
    <member kind="function">
      <type>ArrayView&lt; const char &gt;</type>
      <name>arrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>a57f8d4e0a25951f539461539c90d4e25</anchor>
      <arglist>(Array&lt; const char, D &gt; &amp;array)</arglist>
    </member>
    <member kind="function">
      <type>ArrayView&lt; const const char &gt;</type>
      <name>arrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>a4258433529f847d3e6ed10442f6b9b8b</anchor>
      <arglist>(const Array&lt; const char, D &gt; &amp;array)</arglist>
    </member>
    <member kind="function">
      <type>constexpr ArrayView&lt; const char &gt;</type>
      <name>arrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>a32ef8daa386aee742e5f39fc5b1516fd</anchor>
      <arglist>(const char *data, std::size_t size)</arglist>
    </member>
    <member kind="function">
      <type>constexpr ArrayView&lt; const char &gt;</type>
      <name>arrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>ac8edb9cd0a74f5a0da2ef9c6d03f9ef2</anchor>
      <arglist>(const char(&amp;data)[size])</arglist>
    </member>
    <member kind="function">
      <type>constexpr ArrayView&lt; const char &gt;</type>
      <name>arrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>ac450a49515ff20058552c732542b1b13</anchor>
      <arglist>(StaticArrayView&lt; size, const char &gt; view)</arglist>
    </member>
    <member kind="function">
      <type>constexpr ArrayView&lt; const char &gt;</type>
      <name>arrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>a3a43320e58651eafa9774e04c68545e5</anchor>
      <arglist>(ArrayView&lt; const char &gt; view)</arglist>
    </member>
    <member kind="function">
      <type>constexpr U</type>
      <name>arrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>ade5f07807e9559c80c45e3c314299b91</anchor>
      <arglist>(const char &amp;&amp;other)</arglist>
    </member>
    <member kind="function">
      <type>ArrayView&lt; U &gt;</type>
      <name>arrayCast</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>a8e6435208f2cda38505084a930a3e9ec</anchor>
      <arglist>(ArrayView&lt; const char &gt; view)</arglist>
    </member>
    <member kind="function">
      <type>constexpr std::size_t</type>
      <name>arraySize</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>a19e087600ec3a4f81931386ee28e39ef</anchor>
      <arglist>(ArrayView&lt; const char &gt; view)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>Comparator&lt; Corrade::TestSuite::Compare::Around&lt; T &gt; &gt;</name>
    <filename>classCorrade_1_1TestSuite_1_1Comparator.html</filename>
    <member kind="function">
      <type>ComparisonStatusFlags</type>
      <name>operator()</name>
      <anchorfile>classCorrade_1_1TestSuite_1_1Comparator.html</anchorfile>
      <anchor>a26b5498f872db93256cfd6002bf1a925</anchor>
      <arglist>(const Corrade::TestSuite::Compare::Around&lt; T &gt; &amp;actual, const Corrade::TestSuite::Compare::Around&lt; T &gt; &amp;expected)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>printMessage</name>
      <anchorfile>classCorrade_1_1TestSuite_1_1Comparator.html</anchorfile>
      <anchor>adee45e00d15c7ea1207692cc1bd3248c</anchor>
      <arglist>(ComparisonStatusFlags status, Utility::Debug &amp;out, const char *actual, const char *expected)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>saveDiagnostic</name>
      <anchorfile>classCorrade_1_1TestSuite_1_1Comparator.html</anchorfile>
      <anchor>a089e9fb0d52085b949c388083fecfe63</anchor>
      <arglist>(ComparisonStatusFlags status, Utility::Debug &amp;out, const std::string &amp;path)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>Comparator&lt; Corrade::TestSuite::Compare::File &gt;</name>
    <filename>classCorrade_1_1TestSuite_1_1Comparator.html</filename>
    <member kind="function">
      <type>ComparisonStatusFlags</type>
      <name>operator()</name>
      <anchorfile>classCorrade_1_1TestSuite_1_1Comparator.html</anchorfile>
      <anchor>a26b5498f872db93256cfd6002bf1a925</anchor>
      <arglist>(const Corrade::TestSuite::Compare::File &amp;actual, const Corrade::TestSuite::Compare::File &amp;expected)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>printMessage</name>
      <anchorfile>classCorrade_1_1TestSuite_1_1Comparator.html</anchorfile>
      <anchor>adee45e00d15c7ea1207692cc1bd3248c</anchor>
      <arglist>(ComparisonStatusFlags status, Utility::Debug &amp;out, const char *actual, const char *expected)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>saveDiagnostic</name>
      <anchorfile>classCorrade_1_1TestSuite_1_1Comparator.html</anchorfile>
      <anchor>a089e9fb0d52085b949c388083fecfe63</anchor>
      <arglist>(ComparisonStatusFlags status, Utility::Debug &amp;out, const std::string &amp;path)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>Container&lt; T &gt;</name>
    <filename>classCorrade_1_1TestSuite_1_1Compare_1_1Container.html</filename>
  </compound>
  <compound kind="class">
    <name>Corrade::Containers::Array</name>
    <filename>classCorrade_1_1Containers_1_1Array.html</filename>
    <templarg>T</templarg>
    <templarg>D</templarg>
    <member kind="typedef">
      <type>T</type>
      <name>Type</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>ab88bec3b842462e9eea5157a4049a094</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>D</type>
      <name>Deleter</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>a6397e3aabed2ca448871a747e19a7ee7</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Array</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>ae5ab628ed1d689df54ee1e14c9f33e00</anchor>
      <arglist>(std::nullptr_t) noexcept</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Array</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>ab4c2451e1504971dee5076ad43300be8</anchor>
      <arglist>() noexcept</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Array</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>a4fcbcb24d5a462f13e7dc8e138664e31</anchor>
      <arglist>(DefaultInitT, std::size_t size)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Array</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>ada8220cec2457b9e1730b71d24e3e60e</anchor>
      <arglist>(ValueInitT, std::size_t size)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Array</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>ad6a7e145daae045d737003b626cc958d</anchor>
      <arglist>(NoInitT, std::size_t size)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Array</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>a61202d7c6f6b313d44c6c3f3f3b2331e</anchor>
      <arglist>(DirectInitT, std::size_t size, Args &amp;&amp;... args)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Array</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>aa48105e1a6a95c402c274d32d8d213c1</anchor>
      <arglist>(InPlaceInitT, std::initializer_list&lt; T &gt; list)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Array</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>a913499c6a05cd36964ad56d0eaa1f002</anchor>
      <arglist>(std::size_t size)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Array</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>ab527926a8480885ca15d7fd53c4e0cef</anchor>
      <arglist>(T *data, std::size_t size, D deleter=Implementation::DefaultDeleter&lt; D &gt;{}())</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Array</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>a96d5213dcca0193ecd6c8d67217efde2</anchor>
      <arglist>(const Array&lt; T, D &gt; &amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Array</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>ade7103081fcb82b77f40b3a6a0c9bd1f</anchor>
      <arglist>(Array&lt; T, D &gt; &amp;&amp;other) noexcept</arglist>
    </member>
    <member kind="function">
      <type>Array&lt; T, D &gt; &amp;</type>
      <name>operator=</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>ae4fd95d4d6fb93e26fe27daf744fe1f3</anchor>
      <arglist>(const Array&lt; T, D &gt; &amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type>Array&lt; T, D &gt; &amp;</type>
      <name>operator=</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>a41292530583857fa7d7ccfdce00fb3ba</anchor>
      <arglist>(Array&lt; T, D &gt; &amp;&amp;) noexcept</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>operator U</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>af31e57e0d1e4f2e5d93499c757c8b6df</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>operator U</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>aefbd190121eea9f4c4d3ef05ed9f93d6</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>operator bool</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>a2f5760d29e69b7cd6ed9677d7437dc55</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>operator T *</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>ab0bf774c244d0d7e45a0d6451d27639f</anchor>
      <arglist>() &amp;</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>operator const T *</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>acc671fef58ee49ca4771122488f0802f</anchor>
      <arglist>() const &amp;</arglist>
    </member>
    <member kind="function">
      <type>T *</type>
      <name>data</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>a0c46890d99721a859619561f719b4e31</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const T *</type>
      <name>data</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>a33fe38b5e0d633ff71d09ac0680fdb45</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>D</type>
      <name>deleter</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>acb21f4e838440faf1fb27c734faafb41</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>std::size_t</type>
      <name>size</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>af1e4ac498e1951951729ecab48134a34</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>empty</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>ad1cee21a514b977d045ae8886f8f8b75</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>T *</type>
      <name>begin</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>abaad6108087008802cd894a6679e2b94</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const T *</type>
      <name>begin</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>a2cf50e3d4736428b83c245135a961044</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>const T *</type>
      <name>cbegin</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>a51dfe4f115f8ed72485d544ab64cf769</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>T *</type>
      <name>end</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>a339172cd6f805a667dfbebdcefa533b9</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const T *</type>
      <name>end</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>a123d78064aa332f26b476a45ef0e36f4</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>const T *</type>
      <name>cend</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>a1533ba643c00b3fea2077fbeb39b2afb</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>T &amp;</type>
      <name>front</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>af9952bb49bfc14336f8cf74a36972df8</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const T &amp;</type>
      <name>front</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>a4f01b9e40d5c4da7e6c1f9f73975c53b</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>T &amp;</type>
      <name>back</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>a09753cc0bf9113ccc38af90b6046885b</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const T &amp;</type>
      <name>back</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>a94b5206d0207696751a14e3a4cdae0f7</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>ArrayView&lt; T &gt;</type>
      <name>slice</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>a31ed9632bbdf1612906867854df485bd</anchor>
      <arglist>(T *begin, T *end)</arglist>
    </member>
    <member kind="function">
      <type>ArrayView&lt; const T &gt;</type>
      <name>slice</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>a9ca17fbe62f246a0ecddf34c2f7bdda6</anchor>
      <arglist>(const T *begin, const T *end) const</arglist>
    </member>
    <member kind="function">
      <type>ArrayView&lt; T &gt;</type>
      <name>slice</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>aa73a1d7ad0ac1447d8d88bf4528fad53</anchor>
      <arglist>(std::size_t begin, std::size_t end)</arglist>
    </member>
    <member kind="function">
      <type>ArrayView&lt; const T &gt;</type>
      <name>slice</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>aa8233e6899d80bdc8b5a29b5d6c06a01</anchor>
      <arglist>(std::size_t begin, std::size_t end) const</arglist>
    </member>
    <member kind="function">
      <type>StaticArrayView&lt; size, T &gt;</type>
      <name>slice</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>a824ad0c5ac1722ca966e69177c136622</anchor>
      <arglist>(T *begin)</arglist>
    </member>
    <member kind="function">
      <type>StaticArrayView&lt; size, const T &gt;</type>
      <name>slice</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>a6601fd7167cd944b0d2a04c5964c7746</anchor>
      <arglist>(const T *begin) const</arglist>
    </member>
    <member kind="function">
      <type>StaticArrayView&lt; size, T &gt;</type>
      <name>slice</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>a3364f1140c218b837780b10b970b326f</anchor>
      <arglist>(std::size_t begin)</arglist>
    </member>
    <member kind="function">
      <type>StaticArrayView&lt; size, const T &gt;</type>
      <name>slice</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>a4a00983e653ca797f8dcbf1cc209ae15</anchor>
      <arglist>(std::size_t begin) const</arglist>
    </member>
    <member kind="function">
      <type>StaticArrayView&lt; end_ - begin_, T &gt;</type>
      <name>slice</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>a4efc7e35b1c6e07f6a0538e70ae74f48</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>StaticArrayView&lt; end_ - begin_, const T &gt;</type>
      <name>slice</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>a7a7d4aff6c7407560052e6a991ce4270</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>ArrayView&lt; T &gt;</type>
      <name>prefix</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>a24be60e39728f7d05ba83b0a8eb506bf</anchor>
      <arglist>(T *end)</arglist>
    </member>
    <member kind="function">
      <type>ArrayView&lt; const T &gt;</type>
      <name>prefix</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>a46534a6cb1a2b3fccad4461b36e16f23</anchor>
      <arglist>(const T *end) const</arglist>
    </member>
    <member kind="function">
      <type>ArrayView&lt; T &gt;</type>
      <name>prefix</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>af67d0fcf514203404db65d818d718208</anchor>
      <arglist>(std::size_t end)</arglist>
    </member>
    <member kind="function">
      <type>ArrayView&lt; const T &gt;</type>
      <name>prefix</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>a72500d94e860cbb4a4f1d74e296529e1</anchor>
      <arglist>(std::size_t end) const</arglist>
    </member>
    <member kind="function">
      <type>StaticArrayView&lt; viewSize, T &gt;</type>
      <name>prefix</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>a09dd6511186942d0a86f932275c932c0</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>StaticArrayView&lt; viewSize, const T &gt;</type>
      <name>prefix</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>a9cd5951dd48d0208a0f2059fd323f62b</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>ArrayView&lt; T &gt;</type>
      <name>suffix</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>a803162c8189d5ce30496dd61d515036b</anchor>
      <arglist>(T *begin)</arglist>
    </member>
    <member kind="function">
      <type>ArrayView&lt; const T &gt;</type>
      <name>suffix</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>a71c394e0726949e796704f63d62a4a86</anchor>
      <arglist>(const T *begin) const</arglist>
    </member>
    <member kind="function">
      <type>ArrayView&lt; T &gt;</type>
      <name>suffix</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>adf6635159fa9c6d4c47c4f98ef0248af</anchor>
      <arglist>(std::size_t begin)</arglist>
    </member>
    <member kind="function">
      <type>ArrayView&lt; const T &gt;</type>
      <name>suffix</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>aad441eb3bb38a897fc974dbd95440609</anchor>
      <arglist>(std::size_t begin) const</arglist>
    </member>
    <member kind="function">
      <type>ArrayView&lt; T &gt;</type>
      <name>except</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>a23d4bc894026561c0aea1ec22d5f6350</anchor>
      <arglist>(std::size_t count)</arglist>
    </member>
    <member kind="function">
      <type>ArrayView&lt; const T &gt;</type>
      <name>except</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>a300375138e0e41448e54cedd9982a485</anchor>
      <arglist>(std::size_t count) const</arglist>
    </member>
    <member kind="function">
      <type>T *</type>
      <name>release</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>aa6256e53419003ba6f434a91f293c92e</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>ArrayView&lt; U &gt;</type>
      <name>arrayCast</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>ad5f0615634b2553b0cb304aa58da3347</anchor>
      <arglist>(Array&lt; T, D &gt; &amp;array)</arglist>
    </member>
    <member kind="function">
      <type>std::size_t</type>
      <name>arraySize</name>
      <anchorfile>classCorrade_1_1Containers_1_1Array.html</anchorfile>
      <anchor>af8a4e6b17dca34a5eb82b6a8da8c398b</anchor>
      <arglist>(const Array&lt; T &gt; &amp;view)</arglist>
    </member>
    <docanchor file="classCorrade_1_1Containers_1_1Array" title="Array initialization">Containers-Array-initialization</docanchor>
    <docanchor file="classCorrade_1_1Containers_1_1Array" title="Wrapping externally allocated arrays">Containers-Array-wrapping</docanchor>
    <docanchor file="classCorrade_1_1Containers_1_1Array" title="Conversion to array views">Containers-Array-views</docanchor>
    <docanchor file="classCorrade_1_1Containers_1_1Array" title="STL compatibility">Containers-Array-stl</docanchor>
  </compound>
  <compound kind="class">
    <name>Corrade::Containers::ArrayView</name>
    <filename>classCorrade_1_1Containers_1_1ArrayView.html</filename>
    <templarg>T</templarg>
    <member kind="typedef">
      <type>T</type>
      <name>Type</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>acffca460a3bea7a4721e765f069587b2</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>ArrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>a832bfa9a5d0aa6791d995a80e60621b6</anchor>
      <arglist>(std::nullptr_t) noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>ArrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>aa5b6bedc075e0698b30846770d50d312</anchor>
      <arglist>() noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>ArrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>a67f28104cc25005a20a4e1501f264336</anchor>
      <arglist>(T *data, std::size_t size) noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>ArrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>a4318ae04782dfafbdbbe764bc3e5f15b</anchor>
      <arglist>(U(&amp;data)[size]) noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>ArrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>a983523ac9454f971954d89363bf044c8</anchor>
      <arglist>(ArrayView&lt; U &gt; view) noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>ArrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>a8c3a0e9e3639eb47f6c99b9d72556bf2</anchor>
      <arglist>(StaticArrayView&lt; size, U &gt; view) noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>ArrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>af9b5913220fc5e7a42c5fa3b7ad561a5</anchor>
      <arglist>(U &amp;&amp;other) noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>operator U</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>a230261d1b1f375d09aa74c5f308a09a2</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>operator bool</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>adf0edfab81a28ba75f8d0baee9116310</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>operator T *</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>af6fb5ae89b589cc272297a04fb587e38</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr T *</type>
      <name>data</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>ad94873c712a5e8e46698936fb8f47b71</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr std::size_t</type>
      <name>size</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>a878a1f4c48b49f36fbc4922951d60b5c</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr bool</type>
      <name>empty</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>a6759e541efd29f1bd590240aede6c9f7</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr T *</type>
      <name>begin</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>a0b8ab5ab09f103c21276d14618a5d6bc</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr T *</type>
      <name>cbegin</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>acb1197d09b279ded50ce6187e04d0c31</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr T *</type>
      <name>end</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>ae5982c4c3b93bf0ea82544b772bdf460</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr T *</type>
      <name>cend</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>a13ec5077592c88962836e169226160db</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>T &amp;</type>
      <name>front</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>a6ffb1b09a34e7f019f1794c75c26c3a1</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>T &amp;</type>
      <name>back</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>a160b0f8ab187867806627e00d69b8395</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr ArrayView&lt; T &gt;</type>
      <name>slice</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>aa0af13696fd7aa2d0199eea76789e809</anchor>
      <arglist>(T *begin, T *end) const</arglist>
    </member>
    <member kind="function">
      <type>constexpr ArrayView&lt; T &gt;</type>
      <name>slice</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>addc4457d3b4ad1457e7bf3b89fd63850</anchor>
      <arglist>(std::size_t begin, std::size_t end) const</arglist>
    </member>
    <member kind="function">
      <type>constexpr StaticArrayView&lt; viewSize, T &gt;</type>
      <name>slice</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>a318f61fb34544495328ea2f796a709b8</anchor>
      <arglist>(T *begin) const</arglist>
    </member>
    <member kind="function">
      <type>constexpr StaticArrayView&lt; viewSize, T &gt;</type>
      <name>slice</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>a0bda35382b3f242ea7807cbf6f4546c6</anchor>
      <arglist>(std::size_t begin) const</arglist>
    </member>
    <member kind="function">
      <type>constexpr StaticArrayView&lt; end_ - begin_, T &gt;</type>
      <name>slice</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>a4d991bb99fef30b7cfa8c027c8c0385e</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr ArrayView&lt; T &gt;</type>
      <name>prefix</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>a90dac489ab8d3b34aeb4a4073ff7471e</anchor>
      <arglist>(T *end) const</arglist>
    </member>
    <member kind="function">
      <type>constexpr ArrayView&lt; T &gt;</type>
      <name>prefix</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>a454aa80cc409514620156ea03c3e1796</anchor>
      <arglist>(std::size_t end) const</arglist>
    </member>
    <member kind="function">
      <type>constexpr StaticArrayView&lt; end_, T &gt;</type>
      <name>prefix</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>a4f9b26a22db78cfaa22111159c626165</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr ArrayView&lt; T &gt;</type>
      <name>suffix</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>a259cb9a3a4cc1ba4d68cd92a3debcb4b</anchor>
      <arglist>(T *begin) const</arglist>
    </member>
    <member kind="function">
      <type>constexpr ArrayView&lt; T &gt;</type>
      <name>suffix</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>a7afdfccf156f9471b9282c38fee2b5b6</anchor>
      <arglist>(std::size_t begin) const</arglist>
    </member>
    <member kind="function">
      <type>constexpr ArrayView&lt; T &gt;</type>
      <name>except</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>a0f293b9e5484a13e7b3a36911d2b97da</anchor>
      <arglist>(std::size_t count) const</arglist>
    </member>
    <member kind="function">
      <type>ArrayView&lt; T &gt;</type>
      <name>arrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>a57f8d4e0a25951f539461539c90d4e25</anchor>
      <arglist>(Array&lt; T, D &gt; &amp;array)</arglist>
    </member>
    <member kind="function">
      <type>ArrayView&lt; const T &gt;</type>
      <name>arrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>a4258433529f847d3e6ed10442f6b9b8b</anchor>
      <arglist>(const Array&lt; T, D &gt; &amp;array)</arglist>
    </member>
    <member kind="function">
      <type>constexpr ArrayView&lt; T &gt;</type>
      <name>arrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>a32ef8daa386aee742e5f39fc5b1516fd</anchor>
      <arglist>(T *data, std::size_t size)</arglist>
    </member>
    <member kind="function">
      <type>constexpr ArrayView&lt; T &gt;</type>
      <name>arrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>ac8edb9cd0a74f5a0da2ef9c6d03f9ef2</anchor>
      <arglist>(T(&amp;data)[size])</arglist>
    </member>
    <member kind="function">
      <type>constexpr ArrayView&lt; T &gt;</type>
      <name>arrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>ac450a49515ff20058552c732542b1b13</anchor>
      <arglist>(StaticArrayView&lt; size, T &gt; view)</arglist>
    </member>
    <member kind="function">
      <type>constexpr ArrayView&lt; T &gt;</type>
      <name>arrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>a3a43320e58651eafa9774e04c68545e5</anchor>
      <arglist>(ArrayView&lt; T &gt; view)</arglist>
    </member>
    <member kind="function">
      <type>constexpr U</type>
      <name>arrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>ade5f07807e9559c80c45e3c314299b91</anchor>
      <arglist>(T &amp;&amp;other)</arglist>
    </member>
    <member kind="function">
      <type>ArrayView&lt; U &gt;</type>
      <name>arrayCast</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>a8e6435208f2cda38505084a930a3e9ec</anchor>
      <arglist>(ArrayView&lt; T &gt; view)</arglist>
    </member>
    <member kind="function">
      <type>constexpr std::size_t</type>
      <name>arraySize</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView.html</anchorfile>
      <anchor>a19e087600ec3a4f81931386ee28e39ef</anchor>
      <arglist>(ArrayView&lt; T &gt; view)</arglist>
    </member>
    <docanchor file="classCorrade_1_1Containers_1_1ArrayView" title="STL compatibility">Containers-ArrayView-stl</docanchor>
    <docanchor file="classCorrade_1_1Containers_1_1ArrayView">Containers-ArrayView-single-header</docanchor>
  </compound>
  <compound kind="class">
    <name>Corrade::Containers::ArrayView&lt; const void &gt;</name>
    <filename>classCorrade_1_1Containers_1_1ArrayView_3_01const_01void_01_4.html</filename>
    <member kind="typedef">
      <type>const void</type>
      <name>Type</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView_3_01const_01void_01_4.html</anchorfile>
      <anchor>a866e1baad53839e2c343d7086735737a</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>ArrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView_3_01const_01void_01_4.html</anchorfile>
      <anchor>a0a6b855a35b55a914ceb39bee62bdb39</anchor>
      <arglist>(std::nullptr_t) noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>ArrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView_3_01const_01void_01_4.html</anchorfile>
      <anchor>a2dff420240c3c957e066de9d446ab0d4</anchor>
      <arglist>() noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>ArrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView_3_01const_01void_01_4.html</anchorfile>
      <anchor>a4b52d12c269029373ae89057494cac6a</anchor>
      <arglist>(const void *data, std::size_t size) noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>ArrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView_3_01const_01void_01_4.html</anchorfile>
      <anchor>a6d3439e67990086c4602766e98da45fc</anchor>
      <arglist>(const T *data, std::size_t size) noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>ArrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView_3_01const_01void_01_4.html</anchorfile>
      <anchor>a706d201f881c928c380395efe8828c77</anchor>
      <arglist>(T(&amp;data)[size]) noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>ArrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView_3_01const_01void_01_4.html</anchorfile>
      <anchor>a2078e2ccd1d07bd2f8c66a66dd49271d</anchor>
      <arglist>(ArrayView&lt; T &gt; array) noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>ArrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView_3_01const_01void_01_4.html</anchorfile>
      <anchor>a37da08f38dda6d06dc36196dbbc6f29a</anchor>
      <arglist>(const StaticArrayView&lt; size, T &gt; &amp;array) noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>ArrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView_3_01const_01void_01_4.html</anchorfile>
      <anchor>a11641a8979025a73077d6cf0701287ef</anchor>
      <arglist>(const T &amp;other) noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>operator bool</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView_3_01const_01void_01_4.html</anchorfile>
      <anchor>a83fadb475e0c0bca590cc6098288b512</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>operator const void *</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView_3_01const_01void_01_4.html</anchorfile>
      <anchor>aff5f471896d60a1fa63b4fbf2f0b6d9a</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr const void *</type>
      <name>data</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView_3_01const_01void_01_4.html</anchorfile>
      <anchor>a4a05731e484156c3653acef2fdce6a55</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr std::size_t</type>
      <name>size</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView_3_01const_01void_01_4.html</anchorfile>
      <anchor>aa385ac9159aacb6f282836948973e6ff</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr bool</type>
      <name>empty</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView_3_01const_01void_01_4.html</anchorfile>
      <anchor>a2a88a10860c9e7378fb84a26585dcc6d</anchor>
      <arglist>() const</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>Corrade::Containers::ArrayView&lt; void &gt;</name>
    <filename>classCorrade_1_1Containers_1_1ArrayView_3_01void_01_4.html</filename>
    <member kind="typedef">
      <type>void</type>
      <name>Type</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView_3_01void_01_4.html</anchorfile>
      <anchor>a61c8369af2a1d2924ebae76047bf93d0</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>ArrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView_3_01void_01_4.html</anchorfile>
      <anchor>ae5a218a2756550266a54f311f80d0390</anchor>
      <arglist>(std::nullptr_t) noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>ArrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView_3_01void_01_4.html</anchorfile>
      <anchor>a3210a7c7f4b419168456d5b071edb71b</anchor>
      <arglist>() noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>ArrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView_3_01void_01_4.html</anchorfile>
      <anchor>a2f097505b8bc62c505aac35d4aec9adf</anchor>
      <arglist>(void *data, std::size_t size) noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>ArrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView_3_01void_01_4.html</anchorfile>
      <anchor>a40f06893bab6163d456d4fad299b31f1</anchor>
      <arglist>(T *data, std::size_t size) noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>ArrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView_3_01void_01_4.html</anchorfile>
      <anchor>aa72b415bd3e3eb08c2f2f71fc59db5e6</anchor>
      <arglist>(T(&amp;data)[size]) noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>ArrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView_3_01void_01_4.html</anchorfile>
      <anchor>a9bde953c2ffd96fd5c375def79de7e24</anchor>
      <arglist>(ArrayView&lt; T &gt; array) noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>ArrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView_3_01void_01_4.html</anchorfile>
      <anchor>a5b32b675a9ccce45dab24774ddf4e0fd</anchor>
      <arglist>(const StaticArrayView&lt; size, T &gt; &amp;array) noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>ArrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView_3_01void_01_4.html</anchorfile>
      <anchor>ab1ac331a6b5cdbac3591180e2767194d</anchor>
      <arglist>(T &amp;&amp;other) noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>operator bool</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView_3_01void_01_4.html</anchorfile>
      <anchor>ab4bff475c4a9d866713d115787930e6e</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>operator void *</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView_3_01void_01_4.html</anchorfile>
      <anchor>acc694d35d5824f50afbc95af46ae352c</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr void *</type>
      <name>data</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView_3_01void_01_4.html</anchorfile>
      <anchor>a4d3154b3ee3c69e37248a0c4fca3d795</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr std::size_t</type>
      <name>size</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView_3_01void_01_4.html</anchorfile>
      <anchor>ac83590a71b13ab29ff1dfe3d94cbbe7c</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr bool</type>
      <name>empty</name>
      <anchorfile>classCorrade_1_1Containers_1_1ArrayView_3_01void_01_4.html</anchorfile>
      <anchor>a1ea23bc214a69c4b18fb44fdc80d0352</anchor>
      <arglist>() const</arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>Corrade::Containers::DefaultInitT</name>
    <filename>structCorrade_1_1Containers_1_1DefaultInitT.html</filename>
  </compound>
  <compound kind="struct">
    <name>Corrade::Containers::DirectInitT</name>
    <filename>structCorrade_1_1Containers_1_1DirectInitT.html</filename>
  </compound>
  <compound kind="class">
    <name>Corrade::Containers::EnumSet</name>
    <filename>classCorrade_1_1Containers_1_1EnumSet.html</filename>
    <templarg>T</templarg>
    <templarg>fullValue</templarg>
    <member kind="enumvalue">
      <name>FullValue</name>
      <anchorfile>classCorrade_1_1Containers_1_1EnumSet.html</anchorfile>
      <anchor>a4bb5d41e20280cfae07a6c6f74e5b689a9e456595199d05472bf7927d329639b8</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>T</type>
      <name>Type</name>
      <anchorfile>classCorrade_1_1Containers_1_1EnumSet.html</anchorfile>
      <anchor>af79d534425b3439095c579944858bb4c</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::underlying_type&lt; T &gt;::type</type>
      <name>UnderlyingType</name>
      <anchorfile>classCorrade_1_1Containers_1_1EnumSet.html</anchorfile>
      <anchor>ae98013f13d62ca0bac69d632905846c6</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>FullValue</name>
      <anchorfile>classCorrade_1_1Containers_1_1EnumSet.html</anchorfile>
      <anchor>a4bb5d41e20280cfae07a6c6f74e5b689a9e456595199d05472bf7927d329639b8</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>EnumSet</name>
      <anchorfile>classCorrade_1_1Containers_1_1EnumSet.html</anchorfile>
      <anchor>ac3820645815d705507170a3e507087b5</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>EnumSet</name>
      <anchorfile>classCorrade_1_1Containers_1_1EnumSet.html</anchorfile>
      <anchor>aff6e8c4b80a660f381b31a2ed5106695</anchor>
      <arglist>(T value)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>EnumSet</name>
      <anchorfile>classCorrade_1_1Containers_1_1EnumSet.html</anchorfile>
      <anchor>adb0d3bd1925b399503dc745e51c245a6</anchor>
      <arglist>(NoInitT)</arglist>
    </member>
    <member kind="function">
      <type>constexpr bool</type>
      <name>operator==</name>
      <anchorfile>classCorrade_1_1Containers_1_1EnumSet.html</anchorfile>
      <anchor>a73828ac4e6d6125500733a71eccd0d94</anchor>
      <arglist>(EnumSet&lt; T, fullValue &gt; other) const</arglist>
    </member>
    <member kind="function">
      <type>constexpr bool</type>
      <name>operator!=</name>
      <anchorfile>classCorrade_1_1Containers_1_1EnumSet.html</anchorfile>
      <anchor>a8ae1892b96c6e781165475388e4cbc53</anchor>
      <arglist>(EnumSet&lt; T, fullValue &gt; other) const</arglist>
    </member>
    <member kind="function">
      <type>constexpr bool</type>
      <name>operator&gt;=</name>
      <anchorfile>classCorrade_1_1Containers_1_1EnumSet.html</anchorfile>
      <anchor>aeb270daa9a8afb3a80e57ef2cb2f8314</anchor>
      <arglist>(EnumSet&lt; T, fullValue &gt; other) const</arglist>
    </member>
    <member kind="function">
      <type>constexpr bool</type>
      <name>operator&lt;=</name>
      <anchorfile>classCorrade_1_1Containers_1_1EnumSet.html</anchorfile>
      <anchor>a3ac067add1634645e3d64803e5751459</anchor>
      <arglist>(EnumSet&lt; T, fullValue &gt; other) const</arglist>
    </member>
    <member kind="function">
      <type>constexpr EnumSet&lt; T, fullValue &gt;</type>
      <name>operator|</name>
      <anchorfile>classCorrade_1_1Containers_1_1EnumSet.html</anchorfile>
      <anchor>a89503e71c491ed8fe7bb9f86f48309c4</anchor>
      <arglist>(EnumSet&lt; T, fullValue &gt; other) const</arglist>
    </member>
    <member kind="function">
      <type>EnumSet&lt; T, fullValue &gt; &amp;</type>
      <name>operator|=</name>
      <anchorfile>classCorrade_1_1Containers_1_1EnumSet.html</anchorfile>
      <anchor>a0570b5800f24317a1b52b27d878b9bfa</anchor>
      <arglist>(EnumSet&lt; T, fullValue &gt; other)</arglist>
    </member>
    <member kind="function">
      <type>constexpr EnumSet&lt; T, fullValue &gt;</type>
      <name>operator &amp;</name>
      <anchorfile>classCorrade_1_1Containers_1_1EnumSet.html</anchorfile>
      <anchor>acb47a914ad8ee78f762d78de3ccd12de</anchor>
      <arglist>(EnumSet&lt; T, fullValue &gt; other) const</arglist>
    </member>
    <member kind="function">
      <type>EnumSet&lt; T, fullValue &gt; &amp;</type>
      <name>operator&amp;=</name>
      <anchorfile>classCorrade_1_1Containers_1_1EnumSet.html</anchorfile>
      <anchor>a882bec7925638c92d8491ce07108f398</anchor>
      <arglist>(EnumSet&lt; T, fullValue &gt; other)</arglist>
    </member>
    <member kind="function">
      <type>constexpr EnumSet&lt; T, fullValue &gt;</type>
      <name>operator^</name>
      <anchorfile>classCorrade_1_1Containers_1_1EnumSet.html</anchorfile>
      <anchor>a8b8c4ac2173541eb7572d66b64931512</anchor>
      <arglist>(EnumSet&lt; T, fullValue &gt; other) const</arglist>
    </member>
    <member kind="function">
      <type>EnumSet&lt; T, fullValue &gt; &amp;</type>
      <name>operator^=</name>
      <anchorfile>classCorrade_1_1Containers_1_1EnumSet.html</anchorfile>
      <anchor>afc5caf6f9260aedc01b5d532e4143e36</anchor>
      <arglist>(EnumSet&lt; T, fullValue &gt; other)</arglist>
    </member>
    <member kind="function">
      <type>constexpr EnumSet&lt; T, fullValue &gt;</type>
      <name>operator~</name>
      <anchorfile>classCorrade_1_1Containers_1_1EnumSet.html</anchorfile>
      <anchor>ae9d66612bd81553d5e9cab2dd00235ea</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>operator bool</name>
      <anchorfile>classCorrade_1_1Containers_1_1EnumSet.html</anchorfile>
      <anchor>aaa72f9d5d7ab6e51d66d3698206e7cc9</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>operator UnderlyingType</name>
      <anchorfile>classCorrade_1_1Containers_1_1EnumSet.html</anchorfile>
      <anchor>a49d01415abacaea3fd08be0a18accc54</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>Utility::Debug &amp;</type>
      <name>enumSetDebugOutput</name>
      <anchorfile>classCorrade_1_1Containers_1_1EnumSet.html</anchorfile>
      <anchor>a12c4ea794fa62c56969b1311e2b5c21b</anchor>
      <arglist>(Utility::Debug &amp;debug, EnumSet&lt; T, fullValue &gt; value, const char *empty, std::initializer_list&lt; T &gt; enums)</arglist>
    </member>
    <docanchor file="classCorrade_1_1Containers_1_1EnumSet">EnumSet-out-of-class-operators</docanchor>
    <docanchor file="classCorrade_1_1Containers_1_1EnumSet">EnumSet-friend-operators</docanchor>
  </compound>
  <compound kind="struct">
    <name>Corrade::Containers::InPlaceInitT</name>
    <filename>structCorrade_1_1Containers_1_1InPlaceInitT.html</filename>
  </compound>
  <compound kind="class">
    <name>Corrade::Containers::LinkedList</name>
    <filename>classCorrade_1_1Containers_1_1LinkedList.html</filename>
    <templarg></templarg>
    <member kind="function">
      <type>constexpr</type>
      <name>LinkedList</name>
      <anchorfile>classCorrade_1_1Containers_1_1LinkedList.html</anchorfile>
      <anchor>aef8b2c6fac22b76d6510b030de0bd7ff</anchor>
      <arglist>() noexcept</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>LinkedList</name>
      <anchorfile>classCorrade_1_1Containers_1_1LinkedList.html</anchorfile>
      <anchor>ae30e2ad2d45ed813d5b6517da99c2c45</anchor>
      <arglist>(const LinkedList&lt; T &gt; &amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>LinkedList</name>
      <anchorfile>classCorrade_1_1Containers_1_1LinkedList.html</anchorfile>
      <anchor>a20a700a56461d5b8a6892ca147384609</anchor>
      <arglist>(LinkedList&lt; T &gt; &amp;&amp;other) noexcept</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~LinkedList</name>
      <anchorfile>classCorrade_1_1Containers_1_1LinkedList.html</anchorfile>
      <anchor>ab9bdae1a80b28b53f4b70edd139c2741</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>LinkedList&lt; T &gt; &amp;</type>
      <name>operator=</name>
      <anchorfile>classCorrade_1_1Containers_1_1LinkedList.html</anchorfile>
      <anchor>a93fe27abbdca0a2bdfffbb0c1c9e52b2</anchor>
      <arglist>(const LinkedList&lt; T &gt; &amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type>LinkedList&lt; T &gt; &amp;</type>
      <name>operator=</name>
      <anchorfile>classCorrade_1_1Containers_1_1LinkedList.html</anchorfile>
      <anchor>a77a0c2633a85000db72a6acf84cfde45</anchor>
      <arglist>(LinkedList&lt; T &gt; &amp;&amp;other)</arglist>
    </member>
    <member kind="function">
      <type>T *</type>
      <name>first</name>
      <anchorfile>classCorrade_1_1Containers_1_1LinkedList.html</anchorfile>
      <anchor>a9d49452ca29d53f6313285e8426ec13a</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>constexpr const T *</type>
      <name>first</name>
      <anchorfile>classCorrade_1_1Containers_1_1LinkedList.html</anchorfile>
      <anchor>ae7f1f5924bd7c08a2533a00c05c4276e</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>T *</type>
      <name>last</name>
      <anchorfile>classCorrade_1_1Containers_1_1LinkedList.html</anchorfile>
      <anchor>abdb5e04bfec6bde39d8a20dd8d4d4674</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>constexpr const T *</type>
      <name>last</name>
      <anchorfile>classCorrade_1_1Containers_1_1LinkedList.html</anchorfile>
      <anchor>a3223de2b9ad136253a00b0c3707d3194</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr bool</type>
      <name>isEmpty</name>
      <anchorfile>classCorrade_1_1Containers_1_1LinkedList.html</anchorfile>
      <anchor>a15cb52dca5c7c77b7a33096b0da57280</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>insert</name>
      <anchorfile>classCorrade_1_1Containers_1_1LinkedList.html</anchorfile>
      <anchor>ab85682283d7ef74f20a2c59a3153f9a1</anchor>
      <arglist>(T *item, T *before=nullptr)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>cut</name>
      <anchorfile>classCorrade_1_1Containers_1_1LinkedList.html</anchorfile>
      <anchor>a43d45cf92d3f88f450047b773e5fe157</anchor>
      <arglist>(T *item)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>move</name>
      <anchorfile>classCorrade_1_1Containers_1_1LinkedList.html</anchorfile>
      <anchor>a39ec134725e0b19f1d9bdfcd5e807973</anchor>
      <arglist>(T *item, T *before)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>erase</name>
      <anchorfile>classCorrade_1_1Containers_1_1LinkedList.html</anchorfile>
      <anchor>a5d42a873dd83fbc0cc0084209536f9be</anchor>
      <arglist>(T *item)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>clear</name>
      <anchorfile>classCorrade_1_1Containers_1_1LinkedList.html</anchorfile>
      <anchor>a83550122caae33c90d11d45d0b747184</anchor>
      <arglist>()</arglist>
    </member>
    <docanchor file="classCorrade_1_1Containers_1_1LinkedList" title="Basic usage">Containers-LinkedList-basic-usage</docanchor>
    <docanchor file="classCorrade_1_1Containers_1_1LinkedList" title="Making advantage of pointer to the list">Containers-LinkedList-list-pointer</docanchor>
    <docanchor file="classCorrade_1_1Containers_1_1LinkedList" title="Using private inheritance">Containers-LinkedList-private-inheritance</docanchor>
    <docanchor file="classCorrade_1_1Containers_1_1LinkedList" title="Memory management">Containers-LinkedList-memory-management</docanchor>
  </compound>
  <compound kind="class">
    <name>Corrade::Containers::LinkedListItem</name>
    <filename>classCorrade_1_1Containers_1_1LinkedListItem.html</filename>
    <templarg></templarg>
    <templarg></templarg>
    <member kind="function">
      <type></type>
      <name>LinkedListItem</name>
      <anchorfile>classCorrade_1_1Containers_1_1LinkedListItem.html</anchorfile>
      <anchor>a61ce16db8160e1d80e881d5761f4752b</anchor>
      <arglist>() noexcept</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>LinkedListItem</name>
      <anchorfile>classCorrade_1_1Containers_1_1LinkedListItem.html</anchorfile>
      <anchor>abe1d3f91dd60c7f44e177f6b4c06c016</anchor>
      <arglist>(const LinkedListItem&lt; Derived, List &gt; &amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>LinkedListItem</name>
      <anchorfile>classCorrade_1_1Containers_1_1LinkedListItem.html</anchorfile>
      <anchor>a5a9b570f0fe78f27598febccdae1ddde</anchor>
      <arglist>(LinkedListItem&lt; Derived, List &gt; &amp;&amp;other)</arglist>
    </member>
    <member kind="function">
      <type>LinkedListItem&lt; Derived, List &gt; &amp;</type>
      <name>operator=</name>
      <anchorfile>classCorrade_1_1Containers_1_1LinkedListItem.html</anchorfile>
      <anchor>a90bcc82eb9c28daf4648d7ae3f820a49</anchor>
      <arglist>(const LinkedListItem&lt; Derived, List &gt; &amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type>LinkedListItem&lt; Derived, List &gt; &amp;</type>
      <name>operator=</name>
      <anchorfile>classCorrade_1_1Containers_1_1LinkedListItem.html</anchorfile>
      <anchor>af9f41d1506c9ef6ef4e4f0bb8f3882ef</anchor>
      <arglist>(LinkedListItem&lt; Derived, List &gt; &amp;&amp;other)</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual</type>
      <name>~LinkedListItem</name>
      <anchorfile>classCorrade_1_1Containers_1_1LinkedListItem.html</anchorfile>
      <anchor>a0863b9d8b2ea61e35ff0aac3780870cb</anchor>
      <arglist>()=0</arglist>
    </member>
    <member kind="function">
      <type>List *</type>
      <name>list</name>
      <anchorfile>classCorrade_1_1Containers_1_1LinkedListItem.html</anchorfile>
      <anchor>ab607f167b90d2b00e42b238240afb36d</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const List *</type>
      <name>list</name>
      <anchorfile>classCorrade_1_1Containers_1_1LinkedListItem.html</anchorfile>
      <anchor>a43c52dbbec0606b5b9d76bb7fa80e427</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>Derived *</type>
      <name>previous</name>
      <anchorfile>classCorrade_1_1Containers_1_1LinkedListItem.html</anchorfile>
      <anchor>a600d8bd1ebea7689159d9c37e1302fc4</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const Derived *</type>
      <name>previous</name>
      <anchorfile>classCorrade_1_1Containers_1_1LinkedListItem.html</anchorfile>
      <anchor>af9bb37285e909fb65e623c3ae3308445</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>Derived *</type>
      <name>next</name>
      <anchorfile>classCorrade_1_1Containers_1_1LinkedListItem.html</anchorfile>
      <anchor>ac878e7e23048b57e2e577f73ff2d4870</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const Derived *</type>
      <name>next</name>
      <anchorfile>classCorrade_1_1Containers_1_1LinkedListItem.html</anchorfile>
      <anchor>a49a7456d1edcc292eb7e0c8a0ea8e3f6</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>erase</name>
      <anchorfile>classCorrade_1_1Containers_1_1LinkedListItem.html</anchorfile>
      <anchor>acb954b145234f6ff5ae43f9f2a951b0e</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" protection="private" virtualness="virtual">
      <type>virtual void</type>
      <name>doErase</name>
      <anchorfile>classCorrade_1_1Containers_1_1LinkedListItem.html</anchorfile>
      <anchor>ad08311da29e0b1bad4ffeaeaa5699c2b</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>Corrade::Containers::NoCreateT</name>
    <filename>structCorrade_1_1Containers_1_1NoCreateT.html</filename>
  </compound>
  <compound kind="struct">
    <name>Corrade::Containers::NoInitT</name>
    <filename>structCorrade_1_1Containers_1_1NoInitT.html</filename>
  </compound>
  <compound kind="struct">
    <name>Corrade::Containers::NullOptT</name>
    <filename>structCorrade_1_1Containers_1_1NullOptT.html</filename>
    <member kind="function">
      <type>Utility::Debug &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>structCorrade_1_1Containers_1_1NullOptT.html</anchorfile>
      <anchor>aa6b2e93a65009b7ac7278b36bcd760c9</anchor>
      <arglist>(Utility::Debug &amp;debug, NullOptT)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>Corrade::Containers::Optional</name>
    <filename>classCorrade_1_1Containers_1_1Optional.html</filename>
    <templarg>T</templarg>
    <member kind="function">
      <type></type>
      <name>Optional</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>a8e6f9f4c19abb7a88932b06d150d80b7</anchor>
      <arglist>(NullOptT=NullOpt) noexcept</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Optional</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>ad6663b92a690b02da84cc8923be9504b</anchor>
      <arglist>(const T &amp;value) noexcept(std::is_nothrow_copy_assignable&lt; T &gt;::value)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Optional</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>ad6113b40aabedc4f6bfc3344abd4a17c</anchor>
      <arglist>(T &amp;&amp;value) noexcept(std::is_nothrow_move_assignable&lt; T &gt;::value)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Optional</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>a0032b85cf2cacff3e4cb6e59387d0fe0</anchor>
      <arglist>(InPlaceInitT, Args &amp;&amp;... args) noexcept(std::is_nothrow_constructible&lt; T, Args &amp;&amp;... &gt;::value)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Optional</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>a0b40ee5091561043f7b3db2105df0521</anchor>
      <arglist>(const U &amp;other) noexcept(std::is_nothrow_copy_constructible&lt; T &gt;::value)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Optional</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>a3b40412d5eabf3aef81e871a1d8800dd</anchor>
      <arglist>(U &amp;&amp;other) noexcept(std::is_nothrow_move_constructible&lt; T &gt;::value)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Optional</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>aa9d04a3b7e025d6428552ee7a7b572af</anchor>
      <arglist>(const Optional&lt; T &gt; &amp;other) noexcept(std::is_nothrow_copy_constructible&lt; T &gt;::value)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Optional</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>a89c8a0260c48815b8938484ae455c592</anchor>
      <arglist>(Optional&lt; T &gt; &amp;&amp;other) noexcept(std::is_nothrow_move_constructible&lt; T &gt;::value)</arglist>
    </member>
    <member kind="function">
      <type>Optional&lt; T &gt; &amp;</type>
      <name>operator=</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>a2dcb464863ceb583033a012f53c3a7ad</anchor>
      <arglist>(const Optional&lt; T &gt; &amp;other) noexcept(std::is_nothrow_copy_assignable&lt; T &gt;::value)</arglist>
    </member>
    <member kind="function">
      <type>Optional&lt; T &gt; &amp;</type>
      <name>operator=</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>a6946a7975796cd2c4e6472e33ee679c6</anchor>
      <arglist>(Optional&lt; T &gt; &amp;&amp;other) noexcept(std::is_nothrow_move_assignable&lt; T &gt;::value)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>operator U</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>a1bdc77d731bcc50845234e56450117e9</anchor>
      <arglist>() const &amp;</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>operator U</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>af1d5a8614a3d524db3263cc18494c9c5</anchor>
      <arglist>() &amp;&amp;</arglist>
    </member>
    <member kind="function">
      <type>Optional&lt; T &gt; &amp;</type>
      <name>operator=</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>a36e2f775d6c9e4a209b24d1dda3f19b6</anchor>
      <arglist>(NullOptT) noexcept</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~Optional</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>aca99a8049370de23ee82834babab94ee</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>operator bool</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>a389cd0c17eac32b9194f922fccc91934</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator==</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>a5845d1d681735a657392eb2b9bdd7e4b</anchor>
      <arglist>(const Optional&lt; T &gt; &amp;other) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator!=</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>afd0801e25b3bf275106b816bb43e27f4</anchor>
      <arglist>(const Optional&lt; T &gt; &amp;other) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator==</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>a54979ab51051dfd5827c7cd59ef9595b</anchor>
      <arglist>(NullOptT) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator!=</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>aace07343ad31c9b87fa8e18f77c1c629</anchor>
      <arglist>(NullOptT) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator==</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>a73e7c08fd5a5dfe4c88ada7f145bc3fe</anchor>
      <arglist>(const T &amp;other) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator!=</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>ad95905fb8f63ad6a56d75aa82c4ad53f</anchor>
      <arglist>(const T &amp;other) const</arglist>
    </member>
    <member kind="function">
      <type>T *</type>
      <name>operator-&gt;</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>a000cfe22c91613c0acfb0816d06a3f5e</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const T *</type>
      <name>operator-&gt;</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>a0816ce03c981b5cf64644973619bc861</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>T &amp;</type>
      <name>operator *</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>aa4ff7c7b3c3f3dbe8e8e686078f344e0</anchor>
      <arglist>() &amp;</arglist>
    </member>
    <member kind="function">
      <type>T &amp;&amp;</type>
      <name>operator *</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>a9168523f342bced22853cd8b583c8d7f</anchor>
      <arglist>() &amp;&amp;</arglist>
    </member>
    <member kind="function">
      <type>const T &amp;</type>
      <name>operator *</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>a19e368fa818aaf235edd59ff53942a6a</anchor>
      <arglist>() const &amp;</arglist>
    </member>
    <member kind="function">
      <type>const T &amp;&amp;</type>
      <name>operator *</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>a559026190509308f4fbf6667bb4a404c</anchor>
      <arglist>() const &amp;&amp;</arglist>
    </member>
    <member kind="function">
      <type>T &amp;</type>
      <name>emplace</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>a28d3d69d7026de71a44797101d29f2ab</anchor>
      <arglist>(Args &amp;&amp;... args)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator==</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>a8404f64fbb8bf4965462b97179e4ebaa</anchor>
      <arglist>(NullOptT, const Optional&lt; T &gt; &amp;b)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator!=</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>a1c553d2af62a154653128d1d9ce46a7f</anchor>
      <arglist>(NullOptT, const Optional&lt; T &gt; &amp;b)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator==</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>aba65d386a0b9f9cfd54919c48bbd87e8</anchor>
      <arglist>(const T &amp;a, const Optional&lt; T &gt; &amp;b)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator!=</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>a19c44b59df82dbebce8c720ec88717f5</anchor>
      <arglist>(const T &amp;a, const Optional&lt; T &gt; &amp;b)</arglist>
    </member>
    <member kind="function">
      <type>Optional&lt; typename std::decay&lt; T &gt;::type &gt;</type>
      <name>optional</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>a71e22f3640eb28dd7b2f7b8339931224</anchor>
      <arglist>(T &amp;&amp;value)</arglist>
    </member>
    <member kind="function">
      <type>Optional&lt; T &gt;</type>
      <name>optional</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>a828d28109eb44c02f39b554a392606e9</anchor>
      <arglist>(Args &amp;&amp;... args)</arglist>
    </member>
    <member kind="function">
      <type>auto</type>
      <name>optional</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>ab0ccf001156df9d35c557ce79a66b927</anchor>
      <arglist>(T &amp;&amp;other) -&gt; decltype(Implementation::DeducedOptionalConverter&lt; typename std::decay&lt; T &gt;::type &gt;::from(std::forward&lt; T &gt;(other)))</arglist>
    </member>
    <member kind="function">
      <type>Utility::Debug &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>abb61d1d1f8de79a08d22cef7ef99cb36</anchor>
      <arglist>(Utility::Debug &amp;debug, const Optional&lt; T &gt; &amp;value)</arglist>
    </member>
    <docanchor file="classCorrade_1_1Containers_1_1Optional" title="STL compatibility">Containers-Optional-stl</docanchor>
  </compound>
  <compound kind="class">
    <name>Corrade::Containers::Pointer</name>
    <filename>classCorrade_1_1Containers_1_1Pointer.html</filename>
    <templarg>T</templarg>
    <member kind="function">
      <type></type>
      <name>Pointer</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>adfca41b457c65ad6f7a20e4da25a551d</anchor>
      <arglist>(std::nullptr_t=nullptr) noexcept</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Pointer</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>a6961c1d838ce004acce16785ec434c91</anchor>
      <arglist>(T *pointer) noexcept</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Pointer</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>afa90d4aa96c05da96180f20e3afda237</anchor>
      <arglist>(InPlaceInitT, Args &amp;&amp;... args)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Pointer</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>a30e35c7e582758f8814c2f6b6801dce8</anchor>
      <arglist>(Pointer&lt; U &gt; &amp;&amp;other) noexcept</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Pointer</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>a4e598da5a54d78e0f6e3802a3678ccde</anchor>
      <arglist>(U &amp;&amp;other) noexcept</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Pointer</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>ab3fc3f6972074b386bf623601dfcbebd</anchor>
      <arglist>(const Pointer&lt; T &gt; &amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Pointer</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>a7cf60434462a0199b5bda1ecf4f80c01</anchor>
      <arglist>(Pointer&lt; T &gt; &amp;&amp;other) noexcept</arglist>
    </member>
    <member kind="function">
      <type>Pointer&lt; T &gt; &amp;</type>
      <name>operator=</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>af50cfd42d6d0cbdd0909f66b7d99c8db</anchor>
      <arglist>(const Pointer&lt; T &gt; &amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type>Pointer&lt; T &gt; &amp;</type>
      <name>operator=</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>ac18ee6f61f98e309e478725c61079a75</anchor>
      <arglist>(Pointer&lt; T &gt; &amp;&amp;other) noexcept</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>operator U</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>a26d98d69b279f2618edf0785882e01e5</anchor>
      <arglist>() &amp;&amp;</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator==</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>a5b3089b6c7e3e57b3ebc712b3a1b65a5</anchor>
      <arglist>(std::nullptr_t) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator!=</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>aded8d11d6271891c1fa62572e59bc6dd</anchor>
      <arglist>(std::nullptr_t) const</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~Pointer</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>a3e4509891d92dde6088f5a73b0881a64</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>operator bool</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>a6f4df0db94fe387105c56d0691a25ac4</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>T *</type>
      <name>get</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>a341521cc065c95861ed82d85e26a33d1</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const T *</type>
      <name>get</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>a98b642e46b44b5a4d10a36e216470783</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>T *</type>
      <name>operator-&gt;</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>a52a73243270252c6f91851b377958e62</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const T *</type>
      <name>operator-&gt;</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>ae225b88b0d07991e9fff931bfab2f5d9</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>T &amp;</type>
      <name>operator *</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>a047a241d186b89fe086929927e38b50a</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const T &amp;</type>
      <name>operator *</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>ab04fa2ee84e6d3c6896b715c95d36a0d</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>reset</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>ac4150a4c7336f33d3e0bdbec832154ab</anchor>
      <arglist>(T *pointer=nullptr)</arglist>
    </member>
    <member kind="function">
      <type>T &amp;</type>
      <name>emplace</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>a76f18ffe7324c11f56cf073ca2737717</anchor>
      <arglist>(Args &amp;&amp;... args)</arglist>
    </member>
    <member kind="function">
      <type>T *</type>
      <name>release</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>aaa8b2ed64ca7afb46c614e7aea7893a9</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator==</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>af9b9640eac6d17650d0cc5111c4a19b6</anchor>
      <arglist>(std::nullptr_t, const Pointer&lt; T &gt; &amp;b)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator!=</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>a141814794f6072316c8fa72afc8c7f76</anchor>
      <arglist>(std::nullptr_t, const Pointer&lt; T &gt; &amp;b)</arglist>
    </member>
    <member kind="function">
      <type>Pointer&lt; T &gt;</type>
      <name>pointer</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>a76b1a48222910763fdc5046b6625090e</anchor>
      <arglist>(T *pointer)</arglist>
    </member>
    <member kind="function">
      <type>auto</type>
      <name>pointer</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>a26b8a841d97121802988aff08bcbb0c6</anchor>
      <arglist>(T &amp;&amp;other) -&gt; decltype(Implementation::DeducedPointerConverter&lt; T &gt;::from(std::move(other)))</arglist>
    </member>
    <member kind="function">
      <type>Pointer&lt; U &gt;</type>
      <name>pointerCast</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>ae1b70d106c3a13b76188c0f7f956cb0c</anchor>
      <arglist>(Pointer&lt; T &gt; &amp;&amp;pointer)</arglist>
    </member>
    <member kind="function">
      <type>Pointer&lt; T &gt;</type>
      <name>pointer</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>ab287dc10fa2591e662d9a9ea611a3dad</anchor>
      <arglist>(Args &amp;&amp;... args)</arglist>
    </member>
    <member kind="function">
      <type>Utility::Debug &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>a9c784a034b942686ae70ce8f5b237e26</anchor>
      <arglist>(Utility::Debug &amp;debug, const Pointer&lt; T &gt; &amp;value)</arglist>
    </member>
    <docanchor file="classCorrade_1_1Containers_1_1Pointer" title="STL compatibility">Containers-Pointer-stl</docanchor>
  </compound>
  <compound kind="class">
    <name>Corrade::Containers::Reference</name>
    <filename>classCorrade_1_1Containers_1_1Reference.html</filename>
    <templarg>T</templarg>
    <member kind="function">
      <type>constexpr</type>
      <name>Reference</name>
      <anchorfile>classCorrade_1_1Containers_1_1Reference.html</anchorfile>
      <anchor>a0421d566b189294de33306844e96d9f7</anchor>
      <arglist>(T &amp;reference) noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>Reference</name>
      <anchorfile>classCorrade_1_1Containers_1_1Reference.html</anchorfile>
      <anchor>a4e6818b1ac252c831d5198b8e8e115a4</anchor>
      <arglist>(U other) noexcept</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Reference</name>
      <anchorfile>classCorrade_1_1Containers_1_1Reference.html</anchorfile>
      <anchor>abf5b9c0a0a35bdf6328b3658081d40ae</anchor>
      <arglist>(T &amp;&amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>Reference</name>
      <anchorfile>classCorrade_1_1Containers_1_1Reference.html</anchorfile>
      <anchor>af8d23ae013554277f6b59ff891df756d</anchor>
      <arglist>(Reference&lt; U &gt; other) noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>operator U</name>
      <anchorfile>classCorrade_1_1Containers_1_1Reference.html</anchorfile>
      <anchor>a752abdaca2a9f9e76bf884aad6703b29</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>operator T &amp;</name>
      <anchorfile>classCorrade_1_1Containers_1_1Reference.html</anchorfile>
      <anchor>a886648e43b21bfc39106afba79daf022</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>operator Reference&lt; const T &gt;</name>
      <anchorfile>classCorrade_1_1Containers_1_1Reference.html</anchorfile>
      <anchor>ac2fd386035105f40e6db5ef640f28e5d</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr T &amp;</type>
      <name>get</name>
      <anchorfile>classCorrade_1_1Containers_1_1Reference.html</anchorfile>
      <anchor>a8891dbfbf35e37be7cd08432dd5bb2a7</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr T *</type>
      <name>operator-&gt;</name>
      <anchorfile>classCorrade_1_1Containers_1_1Reference.html</anchorfile>
      <anchor>aeb149ea11dc647219e66f4facf98acae</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr T &amp;</type>
      <name>operator *</name>
      <anchorfile>classCorrade_1_1Containers_1_1Reference.html</anchorfile>
      <anchor>a0ca220977a45ca99b8e87bc6cb5d8dd8</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>Utility::Debug &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>classCorrade_1_1Containers_1_1Reference.html</anchorfile>
      <anchor>aaefc7f623d9f0c2d68e4f42e79e4b161</anchor>
      <arglist>(Utility::Debug &amp;debug, const Reference&lt; T &gt; &amp;value)</arglist>
    </member>
    <docanchor file="classCorrade_1_1Containers_1_1Reference" title="STL compatibility">Containers-Reference-stl</docanchor>
  </compound>
  <compound kind="class">
    <name>Corrade::Containers::ScopeGuard</name>
    <filename>classCorrade_1_1Containers_1_1ScopeGuard.html</filename>
    <member kind="function">
      <type></type>
      <name>ScopeGuard</name>
      <anchorfile>classCorrade_1_1Containers_1_1ScopeGuard.html</anchorfile>
      <anchor>a1b7d59f95a7b156639c2671102dd8f7f</anchor>
      <arglist>(T handle, Deleter deleter)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>ScopeGuard</name>
      <anchorfile>classCorrade_1_1Containers_1_1ScopeGuard.html</anchorfile>
      <anchor>a00c3d0bd42409200a280127687bd0580</anchor>
      <arglist>(Deleter deleter)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>ScopeGuard</name>
      <anchorfile>classCorrade_1_1Containers_1_1ScopeGuard.html</anchorfile>
      <anchor>af181b4a883075a31cdf935f13beb0ba4</anchor>
      <arglist>(const ScopeGuard &amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>ScopeGuard</name>
      <anchorfile>classCorrade_1_1Containers_1_1ScopeGuard.html</anchorfile>
      <anchor>a965a111e107a688e23bf8468b6bf25e3</anchor>
      <arglist>(ScopeGuard &amp;&amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type>ScopeGuard &amp;</type>
      <name>operator=</name>
      <anchorfile>classCorrade_1_1Containers_1_1ScopeGuard.html</anchorfile>
      <anchor>ad300789a640b1dda8c16091721ff7842</anchor>
      <arglist>(const ScopeGuard &amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type>ScopeGuard &amp;</type>
      <name>operator=</name>
      <anchorfile>classCorrade_1_1Containers_1_1ScopeGuard.html</anchorfile>
      <anchor>a98dd8795095421dcf8440d42efd330cb</anchor>
      <arglist>(ScopeGuard &amp;&amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>release</name>
      <anchorfile>classCorrade_1_1Containers_1_1ScopeGuard.html</anchorfile>
      <anchor>a03fd2918f7b6d050298107592d6c09ef</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~ScopeGuard</name>
      <anchorfile>classCorrade_1_1Containers_1_1ScopeGuard.html</anchorfile>
      <anchor>ae4ed8751135f11348d2a6790ee88b661</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>Corrade::Containers::StaticArray</name>
    <filename>classCorrade_1_1Containers_1_1StaticArray.html</filename>
    <templarg>size_</templarg>
    <templarg></templarg>
    <member kind="enumvalue">
      <name>Size</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArray.html</anchorfile>
      <anchor>a5efe7249a7f674c29805692918c63764a793db039a8e75b1828c2a507e5838126</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>T</type>
      <name>Type</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArray.html</anchorfile>
      <anchor>ad9b13acda5c9963c7e36f7d94f225655</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>Size</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArray.html</anchorfile>
      <anchor>a5efe7249a7f674c29805692918c63764a793db039a8e75b1828c2a507e5838126</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>StaticArray</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArray.html</anchorfile>
      <anchor>a08d107eecec22bd96e22d9fb5d8297b2</anchor>
      <arglist>(DefaultInitT)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>StaticArray</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArray.html</anchorfile>
      <anchor>a1157493c2427106a49cc1ca01b1ef3b8</anchor>
      <arglist>(ValueInitT)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>StaticArray</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArray.html</anchorfile>
      <anchor>aae4233fd0137914175919794303ee9a5</anchor>
      <arglist>(NoInitT)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>StaticArray</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArray.html</anchorfile>
      <anchor>a77d56a397ce7414f5e086e4b9a2fb5cc</anchor>
      <arglist>(DirectInitT, Args &amp;&amp;... args)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>StaticArray</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArray.html</anchorfile>
      <anchor>a62cdc55f9ba8f88ba7355292cc7b8038</anchor>
      <arglist>(InPlaceInitT, Args &amp;&amp;... args)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>StaticArray</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArray.html</anchorfile>
      <anchor>ac41f170965520c13b1b1c9d07c919a7a</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>StaticArray</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArray.html</anchorfile>
      <anchor>aec452637a8ad99dc3874750259743cc1</anchor>
      <arglist>(Args &amp;&amp;... args)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>StaticArray</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArray.html</anchorfile>
      <anchor>a9d798fc00a009a1df428c28fbadb77db</anchor>
      <arglist>(const StaticArray&lt; size_, T &gt; &amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>StaticArray</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArray.html</anchorfile>
      <anchor>ab9193077eeadb0562c692385028255ca</anchor>
      <arglist>(StaticArray&lt; size_, T &gt; &amp;&amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type>StaticArray&lt; size_, T &gt; &amp;</type>
      <name>operator=</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArray.html</anchorfile>
      <anchor>a582c622e6472848f441e482dca40c0ad</anchor>
      <arglist>(const StaticArray&lt; size_, T &gt; &amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type>StaticArray&lt; size_, T &gt; &amp;</type>
      <name>operator=</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArray.html</anchorfile>
      <anchor>ab2b8392b6ccec16fe2694f48ae584b40</anchor>
      <arglist>(StaticArray&lt; size_, T &gt; &amp;&amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>operator U</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArray.html</anchorfile>
      <anchor>ae64dcf43e1867da50b75eff9cf4e4c0b</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>operator U</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArray.html</anchorfile>
      <anchor>a4957164b25fe57fee9e08f7fd5aa30b8</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>operator T *</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArray.html</anchorfile>
      <anchor>a865c379cebc89e4346c8c9810651f2eb</anchor>
      <arglist>() &amp;</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>operator const T *</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArray.html</anchorfile>
      <anchor>a9672861f978d238efea9bef3ae9aa8bf</anchor>
      <arglist>() const &amp;</arglist>
    </member>
    <member kind="function">
      <type>T *</type>
      <name>data</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArray.html</anchorfile>
      <anchor>ad36bd00f178baa6fcb1dfc8f2b2461fb</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const T *</type>
      <name>data</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArray.html</anchorfile>
      <anchor>a9472448c92fb04b458d5569dc25ebbd1</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr std::size_t</type>
      <name>size</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArray.html</anchorfile>
      <anchor>a41e0261133d4f749f303cd162568dbe4</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr bool</type>
      <name>empty</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArray.html</anchorfile>
      <anchor>adfa4ea7854df8de503dd80cb419884ac</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>T *</type>
      <name>begin</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArray.html</anchorfile>
      <anchor>a1ae548f4a2509c8c1c4a25a02075e52c</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const T *</type>
      <name>begin</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArray.html</anchorfile>
      <anchor>a33edd30d5450681c273fb9826a3d960b</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>const T *</type>
      <name>cbegin</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArray.html</anchorfile>
      <anchor>acbedfc8ef519f591c185c18b6839a8cf</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>T *</type>
      <name>end</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArray.html</anchorfile>
      <anchor>a5c5506da630022bfa4a92daaadf5d5d4</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const T *</type>
      <name>end</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArray.html</anchorfile>
      <anchor>a7d118529b4c19452906c925a4b5c9374</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>const T *</type>
      <name>cend</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArray.html</anchorfile>
      <anchor>a3f3f2b89e36de45614418e1c81827518</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>T &amp;</type>
      <name>front</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArray.html</anchorfile>
      <anchor>a1491d5812540497153e321aab5769f32</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const T &amp;</type>
      <name>front</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArray.html</anchorfile>
      <anchor>a452ad29b355cc37f572dc2f7c31e8a84</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>T &amp;</type>
      <name>back</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArray.html</anchorfile>
      <anchor>a99b0fe57c62f465b1509a5173adc05b6</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const T &amp;</type>
      <name>back</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArray.html</anchorfile>
      <anchor>a63886a151093e8da913fda15704ccf03</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>ArrayView&lt; T &gt;</type>
      <name>slice</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArray.html</anchorfile>
      <anchor>a79930f8ca1d65cbe71976589adc5e367</anchor>
      <arglist>(T *begin, T *end)</arglist>
    </member>
    <member kind="function">
      <type>ArrayView&lt; const T &gt;</type>
      <name>slice</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArray.html</anchorfile>
      <anchor>af14314de477a94af46a40bcb87d7f674</anchor>
      <arglist>(const T *begin, const T *end) const</arglist>
    </member>
    <member kind="function">
      <type>ArrayView&lt; T &gt;</type>
      <name>slice</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArray.html</anchorfile>
      <anchor>a2f5a20be2e89d05cddc0043e204fd4d2</anchor>
      <arglist>(std::size_t begin, std::size_t end)</arglist>
    </member>
    <member kind="function">
      <type>ArrayView&lt; const T &gt;</type>
      <name>slice</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArray.html</anchorfile>
      <anchor>ab4ff0fb5b9781dd32a9578d2ec60b524</anchor>
      <arglist>(std::size_t begin, std::size_t end) const</arglist>
    </member>
    <member kind="function">
      <type>StaticArrayView&lt; viewSize, T &gt;</type>
      <name>slice</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArray.html</anchorfile>
      <anchor>aafd695442305eac55f636b1905988d13</anchor>
      <arglist>(T *begin)</arglist>
    </member>
    <member kind="function">
      <type>StaticArrayView&lt; viewSize, const T &gt;</type>
      <name>slice</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArray.html</anchorfile>
      <anchor>a48869cf70d232b6206103d8ba7f390a0</anchor>
      <arglist>(const T *begin) const</arglist>
    </member>
    <member kind="function">
      <type>StaticArrayView&lt; viewSize, T &gt;</type>
      <name>slice</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArray.html</anchorfile>
      <anchor>a3405a0f3f83ec4947ac58e56f2388482</anchor>
      <arglist>(std::size_t begin)</arglist>
    </member>
    <member kind="function">
      <type>StaticArrayView&lt; viewSize, const T &gt;</type>
      <name>slice</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArray.html</anchorfile>
      <anchor>a8e83672b0833c216d9712103cae72646</anchor>
      <arglist>(std::size_t begin) const</arglist>
    </member>
    <member kind="function">
      <type>StaticArrayView&lt; end_ - begin_, T &gt;</type>
      <name>slice</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArray.html</anchorfile>
      <anchor>ae672af9c04935db6384849c6ce0872d0</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>StaticArrayView&lt; end_ - begin_, const T &gt;</type>
      <name>slice</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArray.html</anchorfile>
      <anchor>a0a713dee4606401b7ff5ceb832c0a037</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>ArrayView&lt; T &gt;</type>
      <name>prefix</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArray.html</anchorfile>
      <anchor>a47fc19252e6719d0d53cb80c70523875</anchor>
      <arglist>(T *end)</arglist>
    </member>
    <member kind="function">
      <type>ArrayView&lt; const T &gt;</type>
      <name>prefix</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArray.html</anchorfile>
      <anchor>ae105eb6e7c7fa72448cbe872a7699289</anchor>
      <arglist>(const T *end) const</arglist>
    </member>
    <member kind="function">
      <type>ArrayView&lt; T &gt;</type>
      <name>prefix</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArray.html</anchorfile>
      <anchor>ad7042c5d4d41d36f37234419f42bd93e</anchor>
      <arglist>(std::size_t end)</arglist>
    </member>
    <member kind="function">
      <type>ArrayView&lt; const T &gt;</type>
      <name>prefix</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArray.html</anchorfile>
      <anchor>a59863bc77cbc0e0d0904839aac6dc542</anchor>
      <arglist>(std::size_t end) const</arglist>
    </member>
    <member kind="function">
      <type>StaticArrayView&lt; viewSize, T &gt;</type>
      <name>prefix</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArray.html</anchorfile>
      <anchor>ad69b5078da1eba693f02dc8166a76333</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>StaticArrayView&lt; viewSize, const T &gt;</type>
      <name>prefix</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArray.html</anchorfile>
      <anchor>a4d673e8cab3962d47ec5bbda0f8ce7d4</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>ArrayView&lt; T &gt;</type>
      <name>suffix</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArray.html</anchorfile>
      <anchor>aac621f1a78ed76071622d73a584d69a3</anchor>
      <arglist>(T *begin)</arglist>
    </member>
    <member kind="function">
      <type>ArrayView&lt; const T &gt;</type>
      <name>suffix</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArray.html</anchorfile>
      <anchor>a094c5def39f369b513014eaf11f5002f</anchor>
      <arglist>(const T *begin) const</arglist>
    </member>
    <member kind="function">
      <type>ArrayView&lt; T &gt;</type>
      <name>suffix</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArray.html</anchorfile>
      <anchor>a886b160fc0e823ac4542eb1ad4ca63c6</anchor>
      <arglist>(std::size_t begin)</arglist>
    </member>
    <member kind="function">
      <type>ArrayView&lt; const T &gt;</type>
      <name>suffix</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArray.html</anchorfile>
      <anchor>a54f923433bebd94efd9baf56ea481651</anchor>
      <arglist>(std::size_t begin) const</arglist>
    </member>
    <member kind="function">
      <type>StaticArrayView&lt; size_ - begin_, T &gt;</type>
      <name>suffix</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArray.html</anchorfile>
      <anchor>a690d98451f7cd306a157bf987e7dd212</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>StaticArrayView&lt; size_ - begin_, const T &gt;</type>
      <name>suffix</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArray.html</anchorfile>
      <anchor>a6cdba11012638927e5917039793a828f</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>ArrayView&lt; T &gt;</type>
      <name>except</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArray.html</anchorfile>
      <anchor>adb35df6d32598b0731ad92936e06205d</anchor>
      <arglist>(std::size_t count)</arglist>
    </member>
    <member kind="function">
      <type>ArrayView&lt; const T &gt;</type>
      <name>except</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArray.html</anchorfile>
      <anchor>a4270aef389651ca4555753dd2f69b4c8</anchor>
      <arglist>(std::size_t count) const</arglist>
    </member>
    <member kind="function">
      <type>StaticArrayView&lt; size_ - count, T &gt;</type>
      <name>except</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArray.html</anchorfile>
      <anchor>a8f8698347ac49d63f388f88fc716ec5d</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>StaticArrayView&lt; size_ - count, const T &gt;</type>
      <name>except</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArray.html</anchorfile>
      <anchor>ad9be9048afe4879aa65e02550bd6c41c</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr ArrayView&lt; T &gt;</type>
      <name>arrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArray.html</anchorfile>
      <anchor>aa8111199cdb6cc30c470e3cc9c54a30e</anchor>
      <arglist>(StaticArray&lt; size, T &gt; &amp;array)</arglist>
    </member>
    <member kind="function">
      <type>constexpr ArrayView&lt; const T &gt;</type>
      <name>arrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArray.html</anchorfile>
      <anchor>a5b40a450ade4927381bf97a165e3621e</anchor>
      <arglist>(const StaticArray&lt; size, T &gt; &amp;array)</arglist>
    </member>
    <member kind="function">
      <type>constexpr StaticArrayView&lt; size, T &gt;</type>
      <name>staticArrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArray.html</anchorfile>
      <anchor>a4f6c86de70a1ab496a1d798648c22f80</anchor>
      <arglist>(StaticArray&lt; size, T &gt; &amp;array)</arglist>
    </member>
    <member kind="function">
      <type>constexpr StaticArrayView&lt; size, const T &gt;</type>
      <name>staticArrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArray.html</anchorfile>
      <anchor>ab15dc352f75f85e562eaa85ae058ac9f</anchor>
      <arglist>(const StaticArray&lt; size, T &gt; &amp;array)</arglist>
    </member>
    <member kind="function">
      <type>StaticArrayView&lt; size *sizeof(T)/sizeof(U), U &gt;</type>
      <name>arrayCast</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArray.html</anchorfile>
      <anchor>a5ebf447d7f388e81fe6d34910c80cfbe</anchor>
      <arglist>(StaticArray&lt; size, T &gt; &amp;array)</arglist>
    </member>
    <member kind="function">
      <type>constexpr std::size_t</type>
      <name>arraySize</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArray.html</anchorfile>
      <anchor>a0cad6e09b65666040bb2460df883e4a3</anchor>
      <arglist>(const StaticArray&lt; size_, T &gt; &amp;)</arglist>
    </member>
    <docanchor file="classCorrade_1_1Containers_1_1StaticArray" title="Array initialization">Containers-StaticArray-initialization</docanchor>
    <docanchor file="classCorrade_1_1Containers_1_1StaticArray" title="Conversion to array views">Containers-StaticArray-views</docanchor>
    <docanchor file="classCorrade_1_1Containers_1_1StaticArray" title="STL compatibility">Containers-StaticArray-stl</docanchor>
  </compound>
  <compound kind="class">
    <name>Corrade::Containers::StaticArrayView</name>
    <filename>classCorrade_1_1Containers_1_1StaticArrayView.html</filename>
    <templarg>size_</templarg>
    <templarg></templarg>
    <member kind="enumvalue">
      <name>Size</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArrayView.html</anchorfile>
      <anchor>a7aab92766227d3c4afb7bd51ad75fb7ca50dc475b85658e4e449517955ac6c5ac</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>T</type>
      <name>Type</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArrayView.html</anchorfile>
      <anchor>a79d34b8f42168ee332f42fd462dfbbc2</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>Size</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArrayView.html</anchorfile>
      <anchor>a7aab92766227d3c4afb7bd51ad75fb7ca50dc475b85658e4e449517955ac6c5ac</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>StaticArrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArrayView.html</anchorfile>
      <anchor>afa86e55be630568e83e24f1ef212ffa7</anchor>
      <arglist>(std::nullptr_t) noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>StaticArrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArrayView.html</anchorfile>
      <anchor>aef1b8c9af2177bf7e0272755ae5cd26f</anchor>
      <arglist>() noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>StaticArrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArrayView.html</anchorfile>
      <anchor>a72c71c4efe6a687e1d0f5165b2100466</anchor>
      <arglist>(T *data) noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>StaticArrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArrayView.html</anchorfile>
      <anchor>a591e233d706fa154872a7eae4d949193</anchor>
      <arglist>(U(&amp;data)[size_]) noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>StaticArrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArrayView.html</anchorfile>
      <anchor>a2edead72560a51aa054885027f2b56c7</anchor>
      <arglist>(StaticArrayView&lt; size_, U &gt; view) noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>StaticArrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArrayView.html</anchorfile>
      <anchor>ae3383f4f7c8841f0579df1186e2b6c84</anchor>
      <arglist>(U &amp;&amp;other) noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>operator U</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArrayView.html</anchorfile>
      <anchor>a942b26e03674f403d49b75b0c4bd212c</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>operator bool</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArrayView.html</anchorfile>
      <anchor>a356e96320af324d7ff63f62e632bf084</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>operator T *</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArrayView.html</anchorfile>
      <anchor>aaf586671f85cc42b5d4f6d88ed6c12df</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr T *</type>
      <name>data</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArrayView.html</anchorfile>
      <anchor>a65f9c2208c10f7b980bfb5f1f62fca37</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr std::size_t</type>
      <name>size</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArrayView.html</anchorfile>
      <anchor>aaf0791568c7d8b1df06766efa91562df</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr bool</type>
      <name>empty</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArrayView.html</anchorfile>
      <anchor>aab36c893ec4a90b0ddd052479bfe23bc</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr T *</type>
      <name>begin</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArrayView.html</anchorfile>
      <anchor>a8295ef9141ccd3abd0b195a2317392a5</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr T *</type>
      <name>cbegin</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArrayView.html</anchorfile>
      <anchor>ab781fd117ff7d4f6ec133d09529b5cb1</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr T *</type>
      <name>end</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArrayView.html</anchorfile>
      <anchor>ae1fa7dc33b3a81bc3557895f2524583b</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr T *</type>
      <name>cend</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArrayView.html</anchorfile>
      <anchor>a99840c2227fa085908fcca6dbafe8123</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>T &amp;</type>
      <name>front</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArrayView.html</anchorfile>
      <anchor>a081f74bf3aeea53bd0675170f76c53fa</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>T &amp;</type>
      <name>back</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArrayView.html</anchorfile>
      <anchor>aecd5dd6f009992c2d00a1c87b55068d8</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr ArrayView&lt; T &gt;</type>
      <name>slice</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArrayView.html</anchorfile>
      <anchor>a29d45fa202e7cde3ae9652ed21379eb1</anchor>
      <arglist>(T *begin, T *end) const</arglist>
    </member>
    <member kind="function">
      <type>constexpr ArrayView&lt; T &gt;</type>
      <name>slice</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArrayView.html</anchorfile>
      <anchor>aa133dfbb9b1dafedf54e80e872a8dadd</anchor>
      <arglist>(std::size_t begin, std::size_t end) const</arglist>
    </member>
    <member kind="function">
      <type>constexpr StaticArrayView&lt; viewSize, T &gt;</type>
      <name>slice</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArrayView.html</anchorfile>
      <anchor>aafd01d9b04bdd4341566c36f0c9300dc</anchor>
      <arglist>(T *begin) const</arglist>
    </member>
    <member kind="function">
      <type>constexpr StaticArrayView&lt; viewSize, T &gt;</type>
      <name>slice</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArrayView.html</anchorfile>
      <anchor>a5177c55eda97cac234a95e4021a860af</anchor>
      <arglist>(std::size_t begin) const</arglist>
    </member>
    <member kind="function">
      <type>constexpr StaticArrayView&lt; end_ - begin_, T &gt;</type>
      <name>slice</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArrayView.html</anchorfile>
      <anchor>afa2366549bf84bb302185806a1f8ff7e</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr ArrayView&lt; T &gt;</type>
      <name>prefix</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArrayView.html</anchorfile>
      <anchor>a5db2e47bb1950100ea0455cbdf9d7314</anchor>
      <arglist>(T *end) const</arglist>
    </member>
    <member kind="function">
      <type>constexpr ArrayView&lt; T &gt;</type>
      <name>prefix</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArrayView.html</anchorfile>
      <anchor>a7ff9e66d388dad6dfc027dd938a88955</anchor>
      <arglist>(std::size_t end) const</arglist>
    </member>
    <member kind="function">
      <type>constexpr StaticArrayView&lt; end_, T &gt;</type>
      <name>prefix</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArrayView.html</anchorfile>
      <anchor>a880bd47fe14242f8862aa7590cb2b34a</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr ArrayView&lt; T &gt;</type>
      <name>suffix</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArrayView.html</anchorfile>
      <anchor>a95c4ea6917ad03481da854253685f764</anchor>
      <arglist>(T *begin) const</arglist>
    </member>
    <member kind="function">
      <type>constexpr ArrayView&lt; T &gt;</type>
      <name>suffix</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArrayView.html</anchorfile>
      <anchor>a3f99acca19638ecae55091a8ec358bd3</anchor>
      <arglist>(std::size_t begin) const</arglist>
    </member>
    <member kind="function">
      <type>constexpr StaticArrayView&lt; size_ - begin_, T &gt;</type>
      <name>suffix</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArrayView.html</anchorfile>
      <anchor>af0b0dece4b1d9f8599c51b56fbc5ec43</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr ArrayView&lt; T &gt;</type>
      <name>except</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArrayView.html</anchorfile>
      <anchor>ab0689f6537c3c70dc0fdd792d6d81bcd</anchor>
      <arglist>(std::size_t count) const</arglist>
    </member>
    <member kind="function">
      <type>constexpr StaticArrayView&lt; size_ - count, T &gt;</type>
      <name>except</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArrayView.html</anchorfile>
      <anchor>a06fa5cb162c29b74418b944309d84d68</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr StaticArrayView&lt; size, T &gt;</type>
      <name>staticArrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArrayView.html</anchorfile>
      <anchor>a58d7bb8d46f44949b57b226485db4a9b</anchor>
      <arglist>(T *data)</arglist>
    </member>
    <member kind="function">
      <type>constexpr StaticArrayView&lt; size, T &gt;</type>
      <name>staticArrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArrayView.html</anchorfile>
      <anchor>a3cf428b256a5ed8941145e4d15ae8b45</anchor>
      <arglist>(T(&amp;data)[size])</arglist>
    </member>
    <member kind="function">
      <type>constexpr StaticArrayView&lt; size, T &gt;</type>
      <name>staticArrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArrayView.html</anchorfile>
      <anchor>aeb96550df22942b8379e77d3942c88c4</anchor>
      <arglist>(StaticArrayView&lt; size, T &gt; view)</arglist>
    </member>
    <member kind="function">
      <type>constexpr U</type>
      <name>staticArrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArrayView.html</anchorfile>
      <anchor>a2f1123798b8bee4ebadf785646108c98</anchor>
      <arglist>(T &amp;&amp;other)</arglist>
    </member>
    <member kind="function">
      <type>StaticArrayView&lt; size *sizeof(T)/sizeof(U), U &gt;</type>
      <name>arrayCast</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArrayView.html</anchorfile>
      <anchor>aee41b58bdb45963c476ce7554dda2290</anchor>
      <arglist>(StaticArrayView&lt; size, T &gt; view)</arglist>
    </member>
    <member kind="function">
      <type>StaticArrayView&lt; size *sizeof(T)/sizeof(U), U &gt;</type>
      <name>arrayCast</name>
      <anchorfile>classCorrade_1_1Containers_1_1StaticArrayView.html</anchorfile>
      <anchor>aee9c43d476128f05403e638ce539c90d</anchor>
      <arglist>(T(&amp;data)[size])</arglist>
    </member>
    <docanchor file="classCorrade_1_1Containers_1_1StaticArrayView" title="STL compatibility">Containers-StaticArrayView-stl</docanchor>
  </compound>
  <compound kind="class">
    <name>Corrade::Containers::StridedArrayView</name>
    <filename>classCorrade_1_1Containers_1_1StridedArrayView.html</filename>
    <templarg>dimensions</templarg>
    <templarg></templarg>
    <member kind="enumvalue">
      <name>Dimensions</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedArrayView.html</anchorfile>
      <anchor>adb5b022506c78d03368dac36e5e29883afe9163bc54c942d382f73916b318e314</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>T</type>
      <name>Type</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedArrayView.html</anchorfile>
      <anchor>abb81f83fa1c939c3e752d50269f61e7b</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::conditional&lt; dimensions==1, T &amp;, StridedArrayView&lt; dimensions - 1, T &gt; &gt;::type</type>
      <name>ElementType</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedArrayView.html</anchorfile>
      <anchor>acca545349b4e2482a87de8db5f942694</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::conditional&lt; std::is_const&lt; T &gt;::value, const void, void &gt;::type</type>
      <name>ErasedType</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedArrayView.html</anchorfile>
      <anchor>a3195af1de409d1632d012511e778c56c</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>StridedDimensions&lt; dimensions, std::size_t &gt;</type>
      <name>Size</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedArrayView.html</anchorfile>
      <anchor>a1b68d4075a2b003c3b7381a1e1d0ef3e</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>StridedDimensions&lt; dimensions, std::ptrdiff_t &gt;</type>
      <name>Stride</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedArrayView.html</anchorfile>
      <anchor>a06b164d937102e231498d949a23dae94</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>Dimensions</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedArrayView.html</anchorfile>
      <anchor>adb5b022506c78d03368dac36e5e29883afe9163bc54c942d382f73916b318e314</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>StridedArrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedArrayView.html</anchorfile>
      <anchor>a8b8a8a898f82ed191cb0e800600d0db2</anchor>
      <arglist>(std::nullptr_t) noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>StridedArrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedArrayView.html</anchorfile>
      <anchor>af96e5d642a9d8cf88eaa6388a8f55a24</anchor>
      <arglist>() noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>StridedArrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedArrayView.html</anchorfile>
      <anchor>a7d2c88ea6ae84366a85f1c6846485f53</anchor>
      <arglist>(Containers::ArrayView&lt; ErasedType &gt; data, T *member, const Size &amp;size, const Stride &amp;stride) noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>StridedArrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedArrayView.html</anchorfile>
      <anchor>a532bdb09ce4ceeaef841185d05a860f3</anchor>
      <arglist>(Containers::ArrayView&lt; T &gt; data, const Size &amp;size, const Stride &amp;stride) noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>StridedArrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedArrayView.html</anchorfile>
      <anchor>a3600122d0621a5853b0a0e1a6c5fd23b</anchor>
      <arglist>(Containers::ArrayView&lt; T &gt; data, const Size &amp;size) noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>StridedArrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedArrayView.html</anchorfile>
      <anchor>afd8d0914a7b63621db049c95e67ea20d</anchor>
      <arglist>(U(&amp;data)[size]) noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>StridedArrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedArrayView.html</anchorfile>
      <anchor>a2ece0806b1aed72aa07ddbbcb5f84947</anchor>
      <arglist>(StridedArrayView&lt; dimensions, U &gt; view) noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>StridedArrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedArrayView.html</anchorfile>
      <anchor>af0b40a344972d6c815a0a62129dca8cc</anchor>
      <arglist>(ArrayView&lt; U &gt; view) noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>StridedArrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedArrayView.html</anchorfile>
      <anchor>a747b319ad0808ffb8787c853bae78991</anchor>
      <arglist>(StaticArrayView&lt; size, U &gt; view) noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>StridedArrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedArrayView.html</anchorfile>
      <anchor>a59bbfaa3d40c6b78effb42efbdbbe6f0</anchor>
      <arglist>(U &amp;&amp;other) noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>operator bool</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedArrayView.html</anchorfile>
      <anchor>a238b21bdb720fdefe5e35b9a0a095093</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr ErasedType *</type>
      <name>data</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedArrayView.html</anchorfile>
      <anchor>a871a799cf0170d403286605fb98a8b16</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr std::conditional&lt; dimensions==1, std::size_t, const Size &amp; &gt;::type</type>
      <name>size</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedArrayView.html</anchorfile>
      <anchor>a5e0efbfda16235973f7e415a11da4aa2</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr std::conditional&lt; dimensions==1, std::ptrdiff_t, const Stride &amp; &gt;::type</type>
      <name>stride</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedArrayView.html</anchorfile>
      <anchor>ab4ebcdd6cf8c77bdbfb3829ef98c7ad5</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr StridedDimensions&lt; dimensions, bool &gt;</type>
      <name>empty</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedArrayView.html</anchorfile>
      <anchor>a2c71260cee808abbdf417f9bf7613348</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>ElementType</type>
      <name>operator[]</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedArrayView.html</anchorfile>
      <anchor>a8f9aff96d63b449c09e8d5156f104abb</anchor>
      <arglist>(std::size_t i) const</arglist>
    </member>
    <member kind="function">
      <type>StridedIterator&lt; dimensions, T &gt;</type>
      <name>begin</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedArrayView.html</anchorfile>
      <anchor>a0c79c72d2d67bbd3969d8576e5f479b5</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>StridedIterator&lt; dimensions, T &gt;</type>
      <name>cbegin</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedArrayView.html</anchorfile>
      <anchor>a568cc60c418120e288351d16b48ec0bf</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>StridedIterator&lt; dimensions, T &gt;</type>
      <name>end</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedArrayView.html</anchorfile>
      <anchor>a0e69750cbb77e23ad2edb68432a9f988</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>StridedIterator&lt; dimensions, T &gt;</type>
      <name>cend</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedArrayView.html</anchorfile>
      <anchor>a4da6f753fb788ad0c8f216bcdf8e99ca</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>ElementType</type>
      <name>front</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedArrayView.html</anchorfile>
      <anchor>a83c222828b4d30da20f63acd4bbd2f37</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>ElementType</type>
      <name>back</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedArrayView.html</anchorfile>
      <anchor>a0ba52c72c40fbdb6edd242a254f30864</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>StridedArrayView&lt; dimensions, T &gt;</type>
      <name>slice</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedArrayView.html</anchorfile>
      <anchor>a82abc341af7a4a4f77d2e1e60b3835d5</anchor>
      <arglist>(std::size_t begin, std::size_t end) const</arglist>
    </member>
    <member kind="function">
      <type>StridedArrayView&lt; newDimensions, T &gt;</type>
      <name>slice</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedArrayView.html</anchorfile>
      <anchor>a98e66a61e521a67d0a11057bc46da0f2</anchor>
      <arglist>(const Size &amp;begin, const Size &amp;end) const</arglist>
    </member>
    <member kind="function">
      <type>StridedArrayView&lt; newDimensions, T &gt;</type>
      <name>slice</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedArrayView.html</anchorfile>
      <anchor>a66c3b7ae7c67cad7896b68202ebff081</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>StridedArrayView&lt; dimensions, T &gt;</type>
      <name>prefix</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedArrayView.html</anchorfile>
      <anchor>a51d42303c282b829eb936156a77ce5b6</anchor>
      <arglist>(std::size_t end) const</arglist>
    </member>
    <member kind="function">
      <type>StridedArrayView&lt; newDimensions, T &gt;</type>
      <name>prefix</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedArrayView.html</anchorfile>
      <anchor>af6262b92c598634dd0867c7f75faa4f5</anchor>
      <arglist>(const Size &amp;end) const</arglist>
    </member>
    <member kind="function">
      <type>StridedArrayView&lt; dimensions, T &gt;</type>
      <name>suffix</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedArrayView.html</anchorfile>
      <anchor>a03c356b4cc5749565168568c22c4a3f9</anchor>
      <arglist>(std::size_t begin) const</arglist>
    </member>
    <member kind="function">
      <type>StridedArrayView&lt; newDimensions, T &gt;</type>
      <name>suffix</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedArrayView.html</anchorfile>
      <anchor>ac7666bb10aae562a9eb75d49d822a58b</anchor>
      <arglist>(const Size &amp;begin) const</arglist>
    </member>
    <member kind="function">
      <type>StridedArrayView&lt; dimensions, T &gt;</type>
      <name>except</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedArrayView.html</anchorfile>
      <anchor>a49510614143fda760b023ac256d0997f</anchor>
      <arglist>(std::size_t count) const</arglist>
    </member>
    <member kind="function">
      <type>StridedArrayView&lt; newDimensions, T &gt;</type>
      <name>except</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedArrayView.html</anchorfile>
      <anchor>a373055b2e038f61b5e3d1cba18d437c5</anchor>
      <arglist>(const Size &amp;count) const</arglist>
    </member>
    <member kind="function">
      <type>StridedArrayView&lt; dimensions, T &gt;</type>
      <name>every</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedArrayView.html</anchorfile>
      <anchor>aec4bf169e413c9f8f0686a2a46292dfa</anchor>
      <arglist>(std::ptrdiff_t skip) const</arglist>
    </member>
    <member kind="function">
      <type>StridedArrayView&lt; dimensions, T &gt;</type>
      <name>every</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedArrayView.html</anchorfile>
      <anchor>a74437f3e7373c65b4f61eb4f36ebb149</anchor>
      <arglist>(const Stride &amp;skip) const</arglist>
    </member>
    <member kind="function">
      <type>StridedArrayView&lt; dimensions, T &gt;</type>
      <name>transposed</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedArrayView.html</anchorfile>
      <anchor>a5feb202064d3836491a5e903c27b4163</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>StridedArrayView&lt; dimensions, T &gt;</type>
      <name>flipped</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedArrayView.html</anchorfile>
      <anchor>a8d85b2b71d3310253795de3dc874adaf</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>StridedArrayView&lt; dimensions, T &gt;</type>
      <name>broadcasted</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedArrayView.html</anchorfile>
      <anchor>ad05cfd4ea7c841ca59948a03ac342795</anchor>
      <arglist>(std::size_t size) const</arglist>
    </member>
    <member kind="function">
      <type>constexpr StridedArrayView1D&lt; T &gt;</type>
      <name>stridedArrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedArrayView.html</anchorfile>
      <anchor>a28f0f3e4514d0a6d4d463f7bd2884f16</anchor>
      <arglist>(T(&amp;data)[size])</arglist>
    </member>
    <member kind="function">
      <type>constexpr StridedArrayView1D&lt; T &gt;</type>
      <name>stridedArrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedArrayView.html</anchorfile>
      <anchor>a8a18ea77b75e5838af7828f4b681de1d</anchor>
      <arglist>(ArrayView&lt; T &gt; view)</arglist>
    </member>
    <member kind="function">
      <type>constexpr StridedArrayView1D&lt; T &gt;</type>
      <name>stridedArrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedArrayView.html</anchorfile>
      <anchor>a5687e5c332876211a23a2202ea89231b</anchor>
      <arglist>(StaticArrayView&lt; size, T &gt; view)</arglist>
    </member>
    <member kind="function">
      <type>constexpr StridedArrayView&lt; dimensions, T &gt;</type>
      <name>stridedArrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedArrayView.html</anchorfile>
      <anchor>ab15f7dff3df81f6529c2b9e401f813dc</anchor>
      <arglist>(StridedArrayView&lt; dimensions, T &gt; view)</arglist>
    </member>
    <member kind="function">
      <type>constexpr U</type>
      <name>stridedArrayView</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedArrayView.html</anchorfile>
      <anchor>af0b91e302ea8ee6b0c76fe99fe6af1f8</anchor>
      <arglist>(T &amp;&amp;other)</arglist>
    </member>
    <member kind="function">
      <type>StridedArrayView&lt; dimensions, U &gt;</type>
      <name>arrayCast</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedArrayView.html</anchorfile>
      <anchor>a529bc0bc79d6272d1e53c85c5ecd7005</anchor>
      <arglist>(const StridedArrayView&lt; dimensions, T &gt; &amp;view)</arglist>
    </member>
    <member kind="function">
      <type>StridedArrayView&lt; newDimensions, U &gt;</type>
      <name>arrayCast</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedArrayView.html</anchorfile>
      <anchor>abb1caf4042587dc306fc92204c441feb</anchor>
      <arglist>(const StridedArrayView&lt; dimensions, T &gt; &amp;view)</arglist>
    </member>
    <docanchor file="classCorrade_1_1Containers_1_1StridedArrayView" title="Multi-dimensional views">Containers-StridedArrayView-multidimensional</docanchor>
    <docanchor file="classCorrade_1_1Containers_1_1StridedArrayView" title="Zero and negative stride">Containers-StridedArrayView-zero-stride</docanchor>
    <docanchor file="classCorrade_1_1Containers_1_1StridedArrayView" title="STL compatibility">Containers-StridedArrayView-stl</docanchor>
  </compound>
  <compound kind="class">
    <name>Corrade::Containers::StridedDimensions</name>
    <filename>classCorrade_1_1Containers_1_1StridedDimensions.html</filename>
    <templarg>dimensions</templarg>
    <templarg>T</templarg>
    <member kind="function">
      <type>constexpr</type>
      <name>StridedDimensions</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedDimensions.html</anchorfile>
      <anchor>a96cb09a101b6639009bfe79cdd9204b4</anchor>
      <arglist>() noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>StridedDimensions</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedDimensions.html</anchorfile>
      <anchor>a9c671335304d3df36b791cf9dfa75ac8</anchor>
      <arglist>(ValueInitT) noexcept</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>StridedDimensions</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedDimensions.html</anchorfile>
      <anchor>a1e59b07b34e98084ec6808d34a58a88f</anchor>
      <arglist>(NoInitT) noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>StridedDimensions</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedDimensions.html</anchorfile>
      <anchor>a50802cad6c511e2cf76e00b0189afab8</anchor>
      <arglist>(T first, Args... next) noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>StridedDimensions</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedDimensions.html</anchorfile>
      <anchor>a484f60e4b07ae0e51e8c5856c6118322</anchor>
      <arglist>(StaticArrayView&lt; dimensions, const T &gt; values) noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>StridedDimensions</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedDimensions.html</anchorfile>
      <anchor>a19e3e994e96fe16d0853dcf7453495c6</anchor>
      <arglist>(const T(&amp;values)[dimensions]) noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>operator StaticArrayView&lt; dimensions, const T &gt;</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedDimensions.html</anchorfile>
      <anchor>a25ca4a3bc4108137283474e5a1b2b67c</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>operator T</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedDimensions.html</anchorfile>
      <anchor>a2451492c97bd37ae9a76ef14a986eb14</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator==</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedDimensions.html</anchorfile>
      <anchor>a7e5c4ebf3c1b92f7e30db2c2a596579b</anchor>
      <arglist>(const StridedDimensions&lt; dimensions, T &gt; &amp;other) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator!=</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedDimensions.html</anchorfile>
      <anchor>a09d527e5798d5e827ac8b7ca3b5def82</anchor>
      <arglist>(const StridedDimensions&lt; dimensions, T &gt; &amp;other) const</arglist>
    </member>
    <member kind="function">
      <type>constexpr T</type>
      <name>operator[]</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedDimensions.html</anchorfile>
      <anchor>a4187e63cd7612bb4dfe370ae7cafda68</anchor>
      <arglist>(std::size_t i) const</arglist>
    </member>
    <member kind="function">
      <type>T &amp;</type>
      <name>operator[]</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedDimensions.html</anchorfile>
      <anchor>a8345d58bb4e8920650421d8de4e7935a</anchor>
      <arglist>(std::size_t i)</arglist>
    </member>
    <member kind="function">
      <type>constexpr const T *</type>
      <name>begin</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedDimensions.html</anchorfile>
      <anchor>a4a517253fa3136144eb9de7ab49e4278</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr const T *</type>
      <name>cbegin</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedDimensions.html</anchorfile>
      <anchor>a59f36423a9202d3f5936ab3fc68fa6be</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr const T *</type>
      <name>end</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedDimensions.html</anchorfile>
      <anchor>acdaf28ed3bcbc4b04b4277e457f3464a</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr const T *</type>
      <name>cend</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedDimensions.html</anchorfile>
      <anchor>a8d5d726c6aa1574a3798787879944a72</anchor>
      <arglist>() const</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>Corrade::Containers::StridedIterator</name>
    <filename>classCorrade_1_1Containers_1_1StridedIterator.html</filename>
    <templarg>dimensions</templarg>
    <templarg></templarg>
    <member kind="typedef">
      <type>T</type>
      <name>Type</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedIterator.html</anchorfile>
      <anchor>a3dc843c3144a91ad8dbcf8cf2abc2e33</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::conditional&lt; dimensions==1, T &amp;, StridedArrayView&lt; dimensions - 1, T &gt; &gt;::type</type>
      <name>ElementType</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedIterator.html</anchorfile>
      <anchor>a0c2c01bb27b80f274e35272ae684ffb1</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator==</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedIterator.html</anchorfile>
      <anchor>a7e26320ef7c7c857d5ad7773e2cc8a2a</anchor>
      <arglist>(StridedIterator&lt; dimensions, T &gt; other) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator!=</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedIterator.html</anchorfile>
      <anchor>a5eca38b34d2b7d9351231775d84df44c</anchor>
      <arglist>(StridedIterator&lt; dimensions, T &gt; other) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator&lt;</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedIterator.html</anchorfile>
      <anchor>a1f68ae237d752c143315f3b8d1881320</anchor>
      <arglist>(StridedIterator&lt; dimensions, T &gt; other) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator&lt;=</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedIterator.html</anchorfile>
      <anchor>a29665cf275c9959c578cfc10b9c25b44</anchor>
      <arglist>(StridedIterator&lt; dimensions, T &gt; other) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator&gt;</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedIterator.html</anchorfile>
      <anchor>a694f0f4e32b044126868b5da9d56b5bf</anchor>
      <arglist>(StridedIterator&lt; dimensions, T &gt; other) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator&gt;=</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedIterator.html</anchorfile>
      <anchor>a404e4d5cba2468ccd7b1987238697e20</anchor>
      <arglist>(StridedIterator&lt; dimensions, T &gt; other) const</arglist>
    </member>
    <member kind="function">
      <type>StridedIterator&lt; dimensions, T &gt;</type>
      <name>operator+</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedIterator.html</anchorfile>
      <anchor>a478ed40890d7cdfbb4b52adf5d6caf17</anchor>
      <arglist>(std::ptrdiff_t i) const</arglist>
    </member>
    <member kind="function">
      <type>StridedIterator&lt; dimensions, T &gt;</type>
      <name>operator-</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedIterator.html</anchorfile>
      <anchor>a5d8f0faa9fb192c84128ddd33eebefe4</anchor>
      <arglist>(std::ptrdiff_t i) const</arglist>
    </member>
    <member kind="function">
      <type>std::ptrdiff_t</type>
      <name>operator-</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedIterator.html</anchorfile>
      <anchor>af94aebcc82cac60a1fd66612c573c649</anchor>
      <arglist>(StridedIterator&lt; dimensions, T &gt; it) const</arglist>
    </member>
    <member kind="function">
      <type>StridedIterator&lt; dimensions, T &gt; &amp;</type>
      <name>operator--</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedIterator.html</anchorfile>
      <anchor>a2762205d1577ffd3b1374df821256b69</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>StridedIterator&lt; dimensions, T &gt; &amp;</type>
      <name>operator++</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedIterator.html</anchorfile>
      <anchor>a3fd5f4ceec7a36fdc247a76383a9ee6d</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>ElementType</type>
      <name>operator *</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedIterator.html</anchorfile>
      <anchor>a4e25ed1f4b3cd28500bafe175b9baa61</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>StridedIterator&lt; dimensions, T &gt;</type>
      <name>operator+</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedIterator.html</anchorfile>
      <anchor>a0b1f639c3f71b9beda5e4f1c3d6511dd</anchor>
      <arglist>(std::ptrdiff_t i, StridedIterator&lt; dimensions, T &gt; it)</arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>Corrade::Containers::ValueInitT</name>
    <filename>structCorrade_1_1Containers_1_1ValueInitT.html</filename>
  </compound>
  <compound kind="class">
    <name>Corrade::Interconnect::Connection</name>
    <filename>classCorrade_1_1Interconnect_1_1Connection.html</filename>
    <member kind="function">
      <type>bool</type>
      <name>isConnected</name>
      <anchorfile>classCorrade_1_1Interconnect_1_1Connection.html</anchorfile>
      <anchor>a80c77c8ea15c6e9d1ca5a4061a69f1c0</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>disconnect</name>
      <anchorfile>classCorrade_1_1Interconnect_1_1Connection.html</anchorfile>
      <anchor>aaf5525583e6564580dcec30122470f13</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>isConnectionPossible</name>
      <anchorfile>classCorrade_1_1Interconnect_1_1Connection.html</anchorfile>
      <anchor>a95318caf850ff96897a9ddc089ed5596</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>connect</name>
      <anchorfile>classCorrade_1_1Interconnect_1_1Connection.html</anchorfile>
      <anchor>a2da3a80eb849dfd30148aa643efc6b70</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>Corrade::Interconnect::Emitter</name>
    <filename>classCorrade_1_1Interconnect_1_1Emitter.html</filename>
    <class kind="class">Corrade::Interconnect::Emitter::Signal</class>
    <member kind="function">
      <type></type>
      <name>Emitter</name>
      <anchorfile>classCorrade_1_1Interconnect_1_1Emitter.html</anchorfile>
      <anchor>acb307bb054059a1b4faff4ced7df8b14</anchor>
      <arglist>(const Emitter &amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Emitter</name>
      <anchorfile>classCorrade_1_1Interconnect_1_1Emitter.html</anchorfile>
      <anchor>a65e30e6dc9875908864c42fbebe31ecc</anchor>
      <arglist>(Emitter &amp;&amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type>Emitter &amp;</type>
      <name>operator=</name>
      <anchorfile>classCorrade_1_1Interconnect_1_1Emitter.html</anchorfile>
      <anchor>a5da79d646791d792941ad516b0fd3205</anchor>
      <arglist>(const Emitter &amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type>Emitter &amp;</type>
      <name>operator=</name>
      <anchorfile>classCorrade_1_1Interconnect_1_1Emitter.html</anchorfile>
      <anchor>a722e415daa51bdba50cf73a8e00838fb</anchor>
      <arglist>(Emitter &amp;&amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>hasSignalConnections</name>
      <anchorfile>classCorrade_1_1Interconnect_1_1Emitter.html</anchorfile>
      <anchor>a917f10ba95ce7bfe454dd089e24ab135</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>hasSignalConnections</name>
      <anchorfile>classCorrade_1_1Interconnect_1_1Emitter.html</anchorfile>
      <anchor>a8c0decced44c2678189fca7131809b53</anchor>
      <arglist>(Signal(Emitter::*signal)(Args...)) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>isConnected</name>
      <anchorfile>classCorrade_1_1Interconnect_1_1Emitter.html</anchorfile>
      <anchor>a5b6364da9f46c887f76ef1a23afab0b9</anchor>
      <arglist>(const Connection &amp;connection) const</arglist>
    </member>
    <member kind="function">
      <type>std::size_t</type>
      <name>signalConnectionCount</name>
      <anchorfile>classCorrade_1_1Interconnect_1_1Emitter.html</anchorfile>
      <anchor>a6dd51ebfee0918df0d2818388597a123</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>std::size_t</type>
      <name>signalConnectionCount</name>
      <anchorfile>classCorrade_1_1Interconnect_1_1Emitter.html</anchorfile>
      <anchor>ad6d834e5338780aa3dcd1173b9f35a5c</anchor>
      <arglist>(Signal(Emitter::*signal)(Args...)) const</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>disconnectSignal</name>
      <anchorfile>classCorrade_1_1Interconnect_1_1Emitter.html</anchorfile>
      <anchor>a5139e1e243d19690de8412be122ce37a</anchor>
      <arglist>(Signal(Emitter::*signal)(Args...))</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>disconnectAllSignals</name>
      <anchorfile>classCorrade_1_1Interconnect_1_1Emitter.html</anchorfile>
      <anchor>afff8419f54c355125c47f631373b9b4e</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>Signal</type>
      <name>emit</name>
      <anchorfile>classCorrade_1_1Interconnect_1_1Emitter.html</anchorfile>
      <anchor>a43d05dc134cd9a7e7d5847b2b190171d</anchor>
      <arglist>(Signal(Emitter::*signal)(Args...), typename Implementation::Identity&lt; Args &gt;::Type... args)</arglist>
    </member>
    <member kind="function">
      <type>Connection</type>
      <name>connect</name>
      <anchorfile>classCorrade_1_1Interconnect_1_1Emitter.html</anchorfile>
      <anchor>afe6ec8e2281aa5a2804094419d135005</anchor>
      <arglist>(EmitterObject &amp;emitter, Interconnect::Emitter::Signal(Emitter::*signal)(Args...), Functor &amp;&amp;slot)</arglist>
    </member>
    <member kind="function">
      <type>Connection</type>
      <name>connect</name>
      <anchorfile>classCorrade_1_1Interconnect_1_1Emitter.html</anchorfile>
      <anchor>a52f95755ab08bad47be07955f641f2cc</anchor>
      <arglist>(EmitterObject &amp;emitter, Interconnect::Emitter::Signal(Emitter::*signal)(Args...), ReceiverObject &amp;receiver, void(Receiver::*slot)(Args...))</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>disconnect</name>
      <anchorfile>classCorrade_1_1Interconnect_1_1Emitter.html</anchorfile>
      <anchor>a08518bbf1547ae48fbd32afeba6fc787</anchor>
      <arglist>(Emitter &amp;emitter, const Connection &amp;connection)</arglist>
    </member>
    <docanchor file="classCorrade_1_1Interconnect_1_1Emitter" title="Implementing signals">Interconnect-Emitter-signals</docanchor>
    <docanchor file="classCorrade_1_1Interconnect_1_1Emitter">Interconnect-Emitter-msvc-icf</docanchor>
    <docanchor file="classCorrade_1_1Interconnect_1_1Emitter" title="Connecting signals to slots">Interconnect-Emitter-connections</docanchor>
    <docanchor file="classCorrade_1_1Interconnect_1_1Emitter" title="Free function, lambda and function object slots">Interconnect-Emitter-free-slots</docanchor>
    <docanchor file="classCorrade_1_1Interconnect_1_1Emitter" title="Member function slots">Interconnect-Emitter-member-slots</docanchor>
  </compound>
  <compound kind="class">
    <name>Corrade::Interconnect::Emitter::Signal</name>
    <filename>classCorrade_1_1Interconnect_1_1Emitter_1_1Signal.html</filename>
  </compound>
  <compound kind="class">
    <name>Corrade::Interconnect::Receiver</name>
    <filename>classCorrade_1_1Interconnect_1_1Receiver.html</filename>
    <member kind="function">
      <type></type>
      <name>Receiver</name>
      <anchorfile>classCorrade_1_1Interconnect_1_1Receiver.html</anchorfile>
      <anchor>a57b82b81813abb257b3c31dc7dd461f3</anchor>
      <arglist>(const Receiver &amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Receiver</name>
      <anchorfile>classCorrade_1_1Interconnect_1_1Receiver.html</anchorfile>
      <anchor>adbe214ae7135e06e2dfdcffa5a5702e4</anchor>
      <arglist>(Receiver &amp;&amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type>Receiver &amp;</type>
      <name>operator=</name>
      <anchorfile>classCorrade_1_1Interconnect_1_1Receiver.html</anchorfile>
      <anchor>aa0a89c026f6d28aa81158697cf949fb0</anchor>
      <arglist>(const Receiver &amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type>Receiver &amp;</type>
      <name>operator=</name>
      <anchorfile>classCorrade_1_1Interconnect_1_1Receiver.html</anchorfile>
      <anchor>a4d8747f9488cf8d87e31f4e4b64c436e</anchor>
      <arglist>(Receiver &amp;&amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>hasSlotConnections</name>
      <anchorfile>classCorrade_1_1Interconnect_1_1Receiver.html</anchorfile>
      <anchor>a3eb58493905333fd122e1da38f6ce772</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>std::size_t</type>
      <name>slotConnectionCount</name>
      <anchorfile>classCorrade_1_1Interconnect_1_1Receiver.html</anchorfile>
      <anchor>a0bc5a5b466fae1e8a2dce456970c952c</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>disconnectAllSlots</name>
      <anchorfile>classCorrade_1_1Interconnect_1_1Receiver.html</anchorfile>
      <anchor>aa16eeb8b069f5008586b8f68ccc6dd7f</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>Corrade::Interconnect::StateMachine</name>
    <filename>classCorrade_1_1Interconnect_1_1StateMachine.html</filename>
    <templarg>states</templarg>
    <templarg>inputs</templarg>
    <templarg></templarg>
    <templarg></templarg>
    <base>Corrade::Interconnect::Emitter</base>
    <member kind="enumvalue">
      <name>StateCount</name>
      <anchorfile>classCorrade_1_1Interconnect_1_1StateMachine.html</anchorfile>
      <anchor>a7dc416d686cdbc7209d5365729c08084ad07f6229f97a406202285d31f475db70</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>InputCount</name>
      <anchorfile>classCorrade_1_1Interconnect_1_1StateMachine.html</anchorfile>
      <anchor>a7dc416d686cdbc7209d5365729c08084a269ea6d765c72e02fd88ac70c3067847</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>StateCount</name>
      <anchorfile>classCorrade_1_1Interconnect_1_1StateMachine.html</anchorfile>
      <anchor>a7dc416d686cdbc7209d5365729c08084ad07f6229f97a406202285d31f475db70</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>InputCount</name>
      <anchorfile>classCorrade_1_1Interconnect_1_1StateMachine.html</anchorfile>
      <anchor>a7dc416d686cdbc7209d5365729c08084a269ea6d765c72e02fd88ac70c3067847</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>StateMachine</name>
      <anchorfile>classCorrade_1_1Interconnect_1_1StateMachine.html</anchorfile>
      <anchor>a1632034558adf149fe2cdee82d44af65</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>State</type>
      <name>current</name>
      <anchorfile>classCorrade_1_1Interconnect_1_1StateMachine.html</anchorfile>
      <anchor>a13eded19d718fac8c7573c7c66a5f5ac</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>addTransitions</name>
      <anchorfile>classCorrade_1_1Interconnect_1_1StateMachine.html</anchorfile>
      <anchor>ab331b1128a353a33a58420f68392d417</anchor>
      <arglist>(std::initializer_list&lt; StateTransition&lt; State, Input &gt;&gt; transitions)</arglist>
    </member>
    <member kind="function">
      <type>StateMachine&lt; states, inputs, State, Input &gt; &amp;</type>
      <name>step</name>
      <anchorfile>classCorrade_1_1Interconnect_1_1StateMachine.html</anchorfile>
      <anchor>a659a4bc7dc0220dafe101cc9314543b8</anchor>
      <arglist>(Input input)</arglist>
    </member>
    <member kind="function">
      <type>Signal</type>
      <name>stepped</name>
      <anchorfile>classCorrade_1_1Interconnect_1_1StateMachine.html</anchorfile>
      <anchor>aad2ec34f5cfc02298bf7591fdb2cf2e6</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>Signal</type>
      <name>entered</name>
      <anchorfile>classCorrade_1_1Interconnect_1_1StateMachine.html</anchorfile>
      <anchor>a2d038d607bcaddc5777c08c655244b51</anchor>
      <arglist>(State previous)</arglist>
    </member>
    <member kind="function">
      <type>Signal</type>
      <name>exited</name>
      <anchorfile>classCorrade_1_1Interconnect_1_1StateMachine.html</anchorfile>
      <anchor>a8439d25865bb82eeb0ff38ae8c27c82e</anchor>
      <arglist>(State next)</arglist>
    </member>
    <docanchor file="classCorrade_1_1Interconnect_1_1StateMachine" title="Basic usage">Interconnect-StateMachine-usage</docanchor>
  </compound>
  <compound kind="class">
    <name>Corrade::Interconnect::StateTransition</name>
    <filename>classCorrade_1_1Interconnect_1_1StateTransition.html</filename>
    <templarg></templarg>
    <templarg></templarg>
    <member kind="function">
      <type>constexpr</type>
      <name>StateTransition</name>
      <anchorfile>classCorrade_1_1Interconnect_1_1StateTransition.html</anchorfile>
      <anchor>a2e1b5ba516ad502534fd93dcc716fefe</anchor>
      <arglist>(State from, Input input, State to)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>Corrade::PluginManager::AbstractManager</name>
    <filename>classCorrade_1_1PluginManager_1_1AbstractManager.html</filename>
    <member kind="function">
      <type></type>
      <name>AbstractManager</name>
      <anchorfile>classCorrade_1_1PluginManager_1_1AbstractManager.html</anchorfile>
      <anchor>a15c48240332b27b1066308f2e08e2857</anchor>
      <arglist>(const AbstractManager &amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>AbstractManager</name>
      <anchorfile>classCorrade_1_1PluginManager_1_1AbstractManager.html</anchorfile>
      <anchor>ac897a1c0583f0f251b69c47ced65a960</anchor>
      <arglist>(AbstractManager &amp;&amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type>AbstractManager &amp;</type>
      <name>operator=</name>
      <anchorfile>classCorrade_1_1PluginManager_1_1AbstractManager.html</anchorfile>
      <anchor>ad1e9417feb5fd524d9f59daaf3c6e1cf</anchor>
      <arglist>(const AbstractManager &amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type>AbstractManager &amp;</type>
      <name>operator=</name>
      <anchorfile>classCorrade_1_1PluginManager_1_1AbstractManager.html</anchorfile>
      <anchor>a498d594eb9f1e99325a1ac2c3c372859</anchor>
      <arglist>(AbstractManager &amp;&amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>pluginInterface</name>
      <anchorfile>classCorrade_1_1PluginManager_1_1AbstractManager.html</anchorfile>
      <anchor>ae98cfc3c1338dd830439adebca8a13f5</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>pluginDirectory</name>
      <anchorfile>classCorrade_1_1PluginManager_1_1AbstractManager.html</anchorfile>
      <anchor>ad166069f10271b2232d6696cbf505595</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setPluginDirectory</name>
      <anchorfile>classCorrade_1_1PluginManager_1_1AbstractManager.html</anchorfile>
      <anchor>a35435cec6070fc9384c7baea2f79723e</anchor>
      <arglist>(std::string directory)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>reloadPluginDirectory</name>
      <anchorfile>classCorrade_1_1PluginManager_1_1AbstractManager.html</anchorfile>
      <anchor>afbcdc551e12ded49ee9f709d1f55ddeb</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setPreferredPlugins</name>
      <anchorfile>classCorrade_1_1PluginManager_1_1AbstractManager.html</anchorfile>
      <anchor>a93baed5663d354c0dd860676474de42c</anchor>
      <arglist>(const std::string &amp;alias, std::initializer_list&lt; std::string &gt; plugins)</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; std::string &gt;</type>
      <name>pluginList</name>
      <anchorfile>classCorrade_1_1PluginManager_1_1AbstractManager.html</anchorfile>
      <anchor>a52ef3125fe33bff432c74ce534bfe777</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; std::string &gt;</type>
      <name>aliasList</name>
      <anchorfile>classCorrade_1_1PluginManager_1_1AbstractManager.html</anchorfile>
      <anchor>aa5d4d1ed996eb71ecfdc0c0af56f1393</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>PluginMetadata *</type>
      <name>metadata</name>
      <anchorfile>classCorrade_1_1PluginManager_1_1AbstractManager.html</anchorfile>
      <anchor>a4717b5c7929f24ac6537ca5898d9472f</anchor>
      <arglist>(const std::string &amp;plugin)</arglist>
    </member>
    <member kind="function">
      <type>const PluginMetadata *</type>
      <name>metadata</name>
      <anchorfile>classCorrade_1_1PluginManager_1_1AbstractManager.html</anchorfile>
      <anchor>a44d69ca8649ad4f9572de904fa968e62</anchor>
      <arglist>(const std::string &amp;plugin) const</arglist>
    </member>
    <member kind="function">
      <type>LoadState</type>
      <name>loadState</name>
      <anchorfile>classCorrade_1_1PluginManager_1_1AbstractManager.html</anchorfile>
      <anchor>aab9390eea07026c62eae5938705adc18</anchor>
      <arglist>(const std::string &amp;plugin) const</arglist>
    </member>
    <member kind="function">
      <type>LoadState</type>
      <name>load</name>
      <anchorfile>classCorrade_1_1PluginManager_1_1AbstractManager.html</anchorfile>
      <anchor>ae9b3cb96444fb9b5eadccf2b368e071b</anchor>
      <arglist>(const std::string &amp;plugin)</arglist>
    </member>
    <member kind="function">
      <type>LoadState</type>
      <name>unload</name>
      <anchorfile>classCorrade_1_1PluginManager_1_1AbstractManager.html</anchorfile>
      <anchor>aad58623cdf43ba4bf439d683bdc0fed3</anchor>
      <arglist>(const std::string &amp;plugin)</arglist>
    </member>
    <member kind="variable" static="yes">
      <type>static const int</type>
      <name>Version</name>
      <anchorfile>classCorrade_1_1PluginManager_1_1AbstractManager.html</anchorfile>
      <anchor>a4d34e4baaa7d38999531ea49ad85cf26</anchor>
      <arglist></arglist>
    </member>
    <member kind="function" protection="protected">
      <type></type>
      <name>~AbstractManager</name>
      <anchorfile>classCorrade_1_1PluginManager_1_1AbstractManager.html</anchorfile>
      <anchor>a8ff98135f8b6356a26732fe0c9c34f10</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>Corrade::PluginManager::AbstractManagingPlugin</name>
    <filename>classCorrade_1_1PluginManager_1_1AbstractManagingPlugin.html</filename>
    <templarg></templarg>
    <base>Corrade::PluginManager::AbstractPlugin</base>
    <member kind="function">
      <type></type>
      <name>AbstractManagingPlugin</name>
      <anchorfile>classCorrade_1_1PluginManager_1_1AbstractManagingPlugin.html</anchorfile>
      <anchor>a7126dbd19b67dacdeb12c61f0543ef67</anchor>
      <arglist>()=default</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>AbstractManagingPlugin</name>
      <anchorfile>classCorrade_1_1PluginManager_1_1AbstractManagingPlugin.html</anchorfile>
      <anchor>a4e2b38042c06d11a3ae4b6c5e7ac8a30</anchor>
      <arglist>(Manager&lt; Interface &gt; &amp;manager)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>AbstractManagingPlugin</name>
      <anchorfile>classCorrade_1_1PluginManager_1_1AbstractManagingPlugin.html</anchorfile>
      <anchor>ab2ff51df35c509180151d970f8ea08f5</anchor>
      <arglist>(AbstractManager &amp;manager, const std::string &amp;plugin)</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>Manager&lt; Interface &gt; *</type>
      <name>manager</name>
      <anchorfile>classCorrade_1_1PluginManager_1_1AbstractManagingPlugin.html</anchorfile>
      <anchor>afeabe42c12f4e6cdc074d4b83a81d91a</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" protection="protected">
      <type>const Manager&lt; Interface &gt; *</type>
      <name>manager</name>
      <anchorfile>classCorrade_1_1PluginManager_1_1AbstractManagingPlugin.html</anchorfile>
      <anchor>a89f5e7d56c788c2065aa367caab5506d</anchor>
      <arglist>() const</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>Corrade::PluginManager::AbstractPlugin</name>
    <filename>classCorrade_1_1PluginManager_1_1AbstractPlugin.html</filename>
    <member kind="function">
      <type></type>
      <name>AbstractPlugin</name>
      <anchorfile>classCorrade_1_1PluginManager_1_1AbstractPlugin.html</anchorfile>
      <anchor>a18b88d542a28a5721a708e489a1a6750</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>AbstractPlugin</name>
      <anchorfile>classCorrade_1_1PluginManager_1_1AbstractPlugin.html</anchorfile>
      <anchor>aaf2fbb0914f222052d0705ef5771d530</anchor>
      <arglist>(AbstractManager &amp;manager, const std::string &amp;plugin)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>AbstractPlugin</name>
      <anchorfile>classCorrade_1_1PluginManager_1_1AbstractPlugin.html</anchorfile>
      <anchor>af5a43e817f57f6a2c47cdb38855bce5c</anchor>
      <arglist>(const AbstractPlugin &amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>AbstractPlugin</name>
      <anchorfile>classCorrade_1_1PluginManager_1_1AbstractPlugin.html</anchorfile>
      <anchor>af04b37ff831cf928ffc54244cd8a95f9</anchor>
      <arglist>(AbstractPlugin &amp;&amp;)=delete</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual</type>
      <name>~AbstractPlugin</name>
      <anchorfile>classCorrade_1_1PluginManager_1_1AbstractPlugin.html</anchorfile>
      <anchor>aabb3f885f8cb64f3d2dd0b1886d7816a</anchor>
      <arglist>()=0</arglist>
    </member>
    <member kind="function">
      <type>AbstractPlugin &amp;</type>
      <name>operator=</name>
      <anchorfile>classCorrade_1_1PluginManager_1_1AbstractPlugin.html</anchorfile>
      <anchor>aa542b4972a06b2cc0a16f5c259687475</anchor>
      <arglist>(const AbstractPlugin &amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type>AbstractPlugin &amp;</type>
      <name>operator=</name>
      <anchorfile>classCorrade_1_1PluginManager_1_1AbstractPlugin.html</anchorfile>
      <anchor>a79f411129d16aeb85cec7b2a757fbdb6</anchor>
      <arglist>(AbstractPlugin &amp;&amp;)=delete</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual bool</type>
      <name>canBeDeleted</name>
      <anchorfile>classCorrade_1_1PluginManager_1_1AbstractPlugin.html</anchorfile>
      <anchor>a2acd3abb80cf4bb96582cca5c6670738</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const std::string &amp;</type>
      <name>plugin</name>
      <anchorfile>classCorrade_1_1PluginManager_1_1AbstractPlugin.html</anchorfile>
      <anchor>a298eb8c8dcd35fac5bbf0cbed7038b54</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>const PluginMetadata *</type>
      <name>metadata</name>
      <anchorfile>classCorrade_1_1PluginManager_1_1AbstractPlugin.html</anchorfile>
      <anchor>ad3a1fb6800354bb1cc12f097ccbe11b9</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>Utility::ConfigurationGroup &amp;</type>
      <name>configuration</name>
      <anchorfile>classCorrade_1_1PluginManager_1_1AbstractPlugin.html</anchorfile>
      <anchor>a383f11826a04301a827be573093de962</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const Utility::ConfigurationGroup &amp;</type>
      <name>configuration</name>
      <anchorfile>classCorrade_1_1PluginManager_1_1AbstractPlugin.html</anchorfile>
      <anchor>a4fbae96a5230e3587b50c7767a39e9e8</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static std::string</type>
      <name>pluginInterface</name>
      <anchorfile>classCorrade_1_1PluginManager_1_1AbstractPlugin.html</anchorfile>
      <anchor>aff83befdca26296681af6a38af8de633</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static std::vector&lt; std::string &gt;</type>
      <name>pluginSearchPaths</name>
      <anchorfile>classCorrade_1_1PluginManager_1_1AbstractPlugin.html</anchorfile>
      <anchor>a0354f07e12d5a853deb305a4aa8d8ce1</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>initialize</name>
      <anchorfile>classCorrade_1_1PluginManager_1_1AbstractPlugin.html</anchorfile>
      <anchor>abc4332a41668303e35c99f665983e356</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>finalize</name>
      <anchorfile>classCorrade_1_1PluginManager_1_1AbstractPlugin.html</anchorfile>
      <anchor>a161dadb17ef0e4d0f5169b8fddb81d52</anchor>
      <arglist>()</arglist>
    </member>
    <docanchor file="classCorrade_1_1PluginManager_1_1AbstractPlugin" title="Subclassing">PluginManager-AbstractPlugin-subclassing</docanchor>
  </compound>
  <compound kind="class">
    <name>Corrade::PluginManager::Manager</name>
    <filename>classCorrade_1_1PluginManager_1_1Manager.html</filename>
    <templarg></templarg>
    <base>Corrade::PluginManager::AbstractManager</base>
    <member kind="function">
      <type></type>
      <name>Manager</name>
      <anchorfile>classCorrade_1_1PluginManager_1_1Manager.html</anchorfile>
      <anchor>a5fa8dd9cecbe01640da7311fcebc0e4c</anchor>
      <arglist>(std::string pluginDirectory={})</arglist>
    </member>
    <member kind="function">
      <type>Containers::Pointer&lt; T &gt;</type>
      <name>instantiate</name>
      <anchorfile>classCorrade_1_1PluginManager_1_1Manager.html</anchorfile>
      <anchor>a2a8dae71c86fe973f1f4234ccd6b3a22</anchor>
      <arglist>(const std::string &amp;plugin)</arglist>
    </member>
    <member kind="function">
      <type>Containers::Pointer&lt; T &gt;</type>
      <name>instance</name>
      <anchorfile>classCorrade_1_1PluginManager_1_1Manager.html</anchorfile>
      <anchor>a2a046228fe66a16eb9cb5f004c5f63d2</anchor>
      <arglist>(const std::string &amp;plugin)</arglist>
    </member>
    <member kind="function">
      <type>Containers::Pointer&lt; T &gt;</type>
      <name>loadAndInstantiate</name>
      <anchorfile>classCorrade_1_1PluginManager_1_1Manager.html</anchorfile>
      <anchor>a01eea3ec95e2c2eab6380c033611bb5f</anchor>
      <arglist>(const std::string &amp;plugin)</arglist>
    </member>
    <docanchor file="classCorrade_1_1PluginManager_1_1Manager" title="Plugin directories">PluginManager-Manager-paths</docanchor>
    <docanchor file="classCorrade_1_1PluginManager_1_1Manager" title="Plugin loading, instantiation and unloading">PluginManager-Manager-reload</docanchor>
    <docanchor file="classCorrade_1_1PluginManager_1_1Manager" title="Plugin-specific data and configuration">PluginManager-Manager-data</docanchor>
  </compound>
  <compound kind="class">
    <name>Corrade::PluginManager::PluginMetadata</name>
    <filename>classCorrade_1_1PluginManager_1_1PluginMetadata.html</filename>
    <member kind="function">
      <type></type>
      <name>PluginMetadata</name>
      <anchorfile>classCorrade_1_1PluginManager_1_1PluginMetadata.html</anchorfile>
      <anchor>a9b02c82173fae3e0355c7af18c8405fa</anchor>
      <arglist>(const PluginMetadata &amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>PluginMetadata</name>
      <anchorfile>classCorrade_1_1PluginManager_1_1PluginMetadata.html</anchorfile>
      <anchor>ae16256b78496d9ac115920f316a3f5e9</anchor>
      <arglist>(PluginMetadata &amp;&amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type>PluginMetadata &amp;</type>
      <name>operator=</name>
      <anchorfile>classCorrade_1_1PluginManager_1_1PluginMetadata.html</anchorfile>
      <anchor>a1587b877adb25856dd2d0ce50a267ab4</anchor>
      <arglist>(const PluginMetadata &amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type>PluginMetadata &amp;</type>
      <name>operator=</name>
      <anchorfile>classCorrade_1_1PluginManager_1_1PluginMetadata.html</anchorfile>
      <anchor>a150130388de0191333820fb98e509b49</anchor>
      <arglist>(PluginMetadata &amp;&amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>name</name>
      <anchorfile>classCorrade_1_1PluginManager_1_1PluginMetadata.html</anchorfile>
      <anchor>ab62b48fa719ba8577e1bd81be9ef3284</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>const std::vector&lt; std::string &gt; &amp;</type>
      <name>depends</name>
      <anchorfile>classCorrade_1_1PluginManager_1_1PluginMetadata.html</anchorfile>
      <anchor>ae3ae84b54001310e69ba7db70d0b3966</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; std::string &gt;</type>
      <name>usedBy</name>
      <anchorfile>classCorrade_1_1PluginManager_1_1PluginMetadata.html</anchorfile>
      <anchor>aff5f7306e7c02a2846270b835efbf806</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>const std::vector&lt; std::string &gt; &amp;</type>
      <name>provides</name>
      <anchorfile>classCorrade_1_1PluginManager_1_1PluginMetadata.html</anchorfile>
      <anchor>a55c7d06d32f3c440ee2c94a6e4146bd1</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>const Utility::ConfigurationGroup &amp;</type>
      <name>data</name>
      <anchorfile>classCorrade_1_1PluginManager_1_1PluginMetadata.html</anchorfile>
      <anchor>a916bc503db59fed1028ec4ce235c21db</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>Utility::ConfigurationGroup &amp;</type>
      <name>configuration</name>
      <anchorfile>classCorrade_1_1PluginManager_1_1PluginMetadata.html</anchorfile>
      <anchor>a8b50e0f370dda93101475937424b35a7</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const Utility::ConfigurationGroup &amp;</type>
      <name>configuration</name>
      <anchorfile>classCorrade_1_1PluginManager_1_1PluginMetadata.html</anchorfile>
      <anchor>a0711d17a1bc901c2f709514bd805f8dd</anchor>
      <arglist>() const</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>Corrade::TestSuite::Comparator</name>
    <filename>classCorrade_1_1TestSuite_1_1Comparator.html</filename>
    <templarg>T</templarg>
    <member kind="function">
      <type>ComparisonStatusFlags</type>
      <name>operator()</name>
      <anchorfile>classCorrade_1_1TestSuite_1_1Comparator.html</anchorfile>
      <anchor>a26b5498f872db93256cfd6002bf1a925</anchor>
      <arglist>(const T &amp;actual, const T &amp;expected)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>printMessage</name>
      <anchorfile>classCorrade_1_1TestSuite_1_1Comparator.html</anchorfile>
      <anchor>adee45e00d15c7ea1207692cc1bd3248c</anchor>
      <arglist>(ComparisonStatusFlags status, Utility::Debug &amp;out, const char *actual, const char *expected)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>saveDiagnostic</name>
      <anchorfile>classCorrade_1_1TestSuite_1_1Comparator.html</anchorfile>
      <anchor>a089e9fb0d52085b949c388083fecfe63</anchor>
      <arglist>(ComparisonStatusFlags status, Utility::Debug &amp;out, const std::string &amp;path)</arglist>
    </member>
    <docanchor file="classCorrade_1_1TestSuite_1_1Comparator" title="Subclassing">TestSuite-Comparator-subclassing</docanchor>
    <docanchor file="classCorrade_1_1TestSuite_1_1Comparator" title="Comparing with pseudo-types">TestSuite-Comparator-pseudo-types</docanchor>
    <docanchor file="classCorrade_1_1TestSuite_1_1Comparator" title="Passing parameters to comparators">TestSuite-Comparator-parameters</docanchor>
    <docanchor file="classCorrade_1_1TestSuite_1_1Comparator" title="Printing additional messages">TestSuite-Comparator-messages</docanchor>
    <docanchor file="classCorrade_1_1TestSuite_1_1Comparator" title="Saving diagnostic files">TestSuite-Comparator-save-diagnostic</docanchor>
  </compound>
  <compound kind="class">
    <name>Corrade::TestSuite::Comparator&lt; double &gt;</name>
    <filename>classCorrade_1_1TestSuite_1_1Comparator_3_01double_01_4.html</filename>
  </compound>
  <compound kind="class">
    <name>Corrade::TestSuite::Comparator&lt; float &gt;</name>
    <filename>classCorrade_1_1TestSuite_1_1Comparator_3_01float_01_4.html</filename>
  </compound>
  <compound kind="class">
    <name>Corrade::TestSuite::Comparator&lt; long double &gt;</name>
    <filename>classCorrade_1_1TestSuite_1_1Comparator_3_01long_01double_01_4.html</filename>
  </compound>
  <compound kind="class">
    <name>Corrade::TestSuite::Compare::Around</name>
    <filename>classCorrade_1_1TestSuite_1_1Compare_1_1Around.html</filename>
    <templarg></templarg>
    <member kind="function">
      <type></type>
      <name>Around</name>
      <anchorfile>classCorrade_1_1TestSuite_1_1Compare_1_1Around.html</anchorfile>
      <anchor>a32bc31da967fea727ba4ef4904ee4cf4</anchor>
      <arglist>(T epsilon)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>Corrade::TestSuite::Compare::Container</name>
    <filename>classCorrade_1_1TestSuite_1_1Compare_1_1Container.html</filename>
    <templarg></templarg>
  </compound>
  <compound kind="class">
    <name>Corrade::TestSuite::Compare::File</name>
    <filename>classCorrade_1_1TestSuite_1_1Compare_1_1File.html</filename>
    <member kind="function">
      <type></type>
      <name>File</name>
      <anchorfile>classCorrade_1_1TestSuite_1_1Compare_1_1File.html</anchorfile>
      <anchor>ab8b861a7787d2e74693a515a14197388</anchor>
      <arglist>(const std::string &amp;pathPrefix={})</arglist>
    </member>
    <docanchor file="classCorrade_1_1TestSuite_1_1Compare_1_1File" title="Saving files for failed comparisons">TestSuite-Compare-File-save-diagnostic</docanchor>
  </compound>
  <compound kind="class">
    <name>Corrade::TestSuite::Compare::FileToString</name>
    <filename>classCorrade_1_1TestSuite_1_1Compare_1_1FileToString.html</filename>
  </compound>
  <compound kind="class">
    <name>Corrade::TestSuite::Compare::Greater</name>
    <filename>classCorrade_1_1TestSuite_1_1Compare_1_1Greater.html</filename>
    <templarg></templarg>
  </compound>
  <compound kind="class">
    <name>Corrade::TestSuite::Compare::GreaterOrEqual</name>
    <filename>classCorrade_1_1TestSuite_1_1Compare_1_1GreaterOrEqual.html</filename>
    <templarg></templarg>
  </compound>
  <compound kind="class">
    <name>Corrade::TestSuite::Compare::Less</name>
    <filename>classCorrade_1_1TestSuite_1_1Compare_1_1Less.html</filename>
    <templarg></templarg>
  </compound>
  <compound kind="class">
    <name>Corrade::TestSuite::Compare::LessOrEqual</name>
    <filename>classCorrade_1_1TestSuite_1_1Compare_1_1LessOrEqual.html</filename>
    <templarg></templarg>
  </compound>
  <compound kind="class">
    <name>Corrade::TestSuite::Compare::SortedContainer</name>
    <filename>classCorrade_1_1TestSuite_1_1Compare_1_1SortedContainer.html</filename>
    <templarg></templarg>
    <base>Container&lt; T &gt;</base>
  </compound>
  <compound kind="class">
    <name>Corrade::TestSuite::Compare::StringToFile</name>
    <filename>classCorrade_1_1TestSuite_1_1Compare_1_1StringToFile.html</filename>
    <docanchor file="classCorrade_1_1TestSuite_1_1Compare_1_1StringToFile" title="Saving files for failed comparisons">TestSuite-Compare-StringToFile-save-diagnostic</docanchor>
  </compound>
  <compound kind="class">
    <name>Corrade::TestSuite::Tester</name>
    <filename>classCorrade_1_1TestSuite_1_1Tester.html</filename>
    <class kind="class">Corrade::TestSuite::Tester::TesterConfiguration</class>
    <member kind="enumeration">
      <type></type>
      <name>BenchmarkType</name>
      <anchorfile>classCorrade_1_1TestSuite_1_1Tester.html</anchorfile>
      <anchor>aded4a627bc1573996e981425775f637d</anchor>
      <arglist></arglist>
      <enumvalue file="classCorrade_1_1TestSuite_1_1Tester.html" anchor="aded4a627bc1573996e981425775f637da7a1920d61156abc05a60135aefe8bc67">Default</enumvalue>
      <enumvalue file="classCorrade_1_1TestSuite_1_1Tester.html" anchor="aded4a627bc1573996e981425775f637da75fa4d137ea5adb50f81fb3c26229797">WallTime</enumvalue>
      <enumvalue file="classCorrade_1_1TestSuite_1_1Tester.html" anchor="aded4a627bc1573996e981425775f637da582bcd601669f4956f55cb20feb2614f">CpuTime</enumvalue>
      <enumvalue file="classCorrade_1_1TestSuite_1_1Tester.html" anchor="aded4a627bc1573996e981425775f637daba0d4b831fbcfdb7788fdfb4e8789207">CpuCycles</enumvalue>
    </member>
    <member kind="enumeration">
      <type></type>
      <name>BenchmarkUnits</name>
      <anchorfile>classCorrade_1_1TestSuite_1_1Tester.html</anchorfile>
      <anchor>a37efcdd6ea7a7ec4a8e4b304218308b4</anchor>
      <arglist></arglist>
      <enumvalue file="classCorrade_1_1TestSuite_1_1Tester.html" anchor="a37efcdd6ea7a7ec4a8e4b304218308b4afba00bdab687ce01136a86bac8bac578">Nanoseconds</enumvalue>
      <enumvalue file="classCorrade_1_1TestSuite_1_1Tester.html" anchor="a37efcdd6ea7a7ec4a8e4b304218308b4ad3240659659cbfa93d781d1510717a66">Cycles</enumvalue>
      <enumvalue file="classCorrade_1_1TestSuite_1_1Tester.html" anchor="a37efcdd6ea7a7ec4a8e4b304218308b4a49cc8e6220245b65cd7d20fc6ccc74f5">Instructions</enumvalue>
      <enumvalue file="classCorrade_1_1TestSuite_1_1Tester.html" anchor="a37efcdd6ea7a7ec4a8e4b304218308b4a600e754f49b68aa0fc90a9cd64eb7051">Bytes</enumvalue>
      <enumvalue file="classCorrade_1_1TestSuite_1_1Tester.html" anchor="a37efcdd6ea7a7ec4a8e4b304218308b4ae93f994f01c537c4e2f7d8528c3eb5e9">Count</enumvalue>
    </member>
    <member kind="typedef">
      <type>Corrade::Utility::Debug</type>
      <name>Debug</name>
      <anchorfile>classCorrade_1_1TestSuite_1_1Tester.html</anchorfile>
      <anchor>a8ef15c8aec41c39215745963a3679199</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>Corrade::Utility::Warning</type>
      <name>Warning</name>
      <anchorfile>classCorrade_1_1TestSuite_1_1Tester.html</anchorfile>
      <anchor>a48afcce4f459f33e54da2c3c45f6e54b</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>Corrade::Utility::Error</type>
      <name>Error</name>
      <anchorfile>classCorrade_1_1TestSuite_1_1Tester.html</anchorfile>
      <anchor>a303019d6658462f4b8b365fde8d8c319</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Tester</name>
      <anchorfile>classCorrade_1_1TestSuite_1_1Tester.html</anchorfile>
      <anchor>ae84d406fe1ec5c18fcb782bcb24c9bef</anchor>
      <arglist>(const TesterConfiguration &amp;configuration=TesterConfiguration{})</arglist>
    </member>
    <member kind="function">
      <type>std::pair&lt; int &amp;, char ** &gt;</type>
      <name>arguments</name>
      <anchorfile>classCorrade_1_1TestSuite_1_1Tester.html</anchorfile>
      <anchor>ad254f71ecc24a166536a99094d504090</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>addTests</name>
      <anchorfile>classCorrade_1_1TestSuite_1_1Tester.html</anchorfile>
      <anchor>aad65989bea7cf70d5579dd65ffa1ce9a</anchor>
      <arglist>(std::initializer_list&lt; void(Derived::*)()&gt; tests)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>addRepeatedTests</name>
      <anchorfile>classCorrade_1_1TestSuite_1_1Tester.html</anchorfile>
      <anchor>a06c40ddf62db02c1575d019b630d7f08</anchor>
      <arglist>(std::initializer_list&lt; void(Derived::*)()&gt; tests, std::size_t repeatCount)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>addTests</name>
      <anchorfile>classCorrade_1_1TestSuite_1_1Tester.html</anchorfile>
      <anchor>a705b618d2aa631f2695d6c2adaf156ef</anchor>
      <arglist>(std::initializer_list&lt; void(Derived::*)()&gt; tests, void(Derived::*setup)(), void(Derived::*teardown)())</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>addRepeatedTests</name>
      <anchorfile>classCorrade_1_1TestSuite_1_1Tester.html</anchorfile>
      <anchor>aaa26e0cc2a3431ad8e5ee139079302c1</anchor>
      <arglist>(std::initializer_list&lt; void(Derived::*)()&gt; tests, std::size_t repeatCount, void(Derived::*setup)(), void(Derived::*teardown)())</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>addInstancedTests</name>
      <anchorfile>classCorrade_1_1TestSuite_1_1Tester.html</anchorfile>
      <anchor>af63e77990b01434052ac1c55fd3bf05a</anchor>
      <arglist>(std::initializer_list&lt; void(Derived::*)()&gt; tests, std::size_t instanceCount)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>addRepeatedInstancedTests</name>
      <anchorfile>classCorrade_1_1TestSuite_1_1Tester.html</anchorfile>
      <anchor>ac0fb10f0917037ac8dba8ca2e7d75ab6</anchor>
      <arglist>(std::initializer_list&lt; void(Derived::*)()&gt; tests, std::size_t repeatCount, std::size_t instanceCount)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>addInstancedTests</name>
      <anchorfile>classCorrade_1_1TestSuite_1_1Tester.html</anchorfile>
      <anchor>aa8e029e34201b65e661fa677d1f3234e</anchor>
      <arglist>(std::initializer_list&lt; void(Derived::*)()&gt; tests, std::size_t instanceCount, void(Derived::*setup)(), void(Derived::*teardown)())</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>addRepeatedInstancedTests</name>
      <anchorfile>classCorrade_1_1TestSuite_1_1Tester.html</anchorfile>
      <anchor>ab56fc0bbd9681c4dddf158c04cc8f836</anchor>
      <arglist>(std::initializer_list&lt; void(Derived::*)()&gt; tests, std::size_t repeatCount, std::size_t instanceCount, void(Derived::*setup)(), void(Derived::*teardown)())</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>addBenchmarks</name>
      <anchorfile>classCorrade_1_1TestSuite_1_1Tester.html</anchorfile>
      <anchor>a424aeb9a3c2180464c155d982ae08389</anchor>
      <arglist>(std::initializer_list&lt; void(Derived::*)()&gt; benchmarks, std::size_t batchCount, BenchmarkType benchmarkType=BenchmarkType::Default)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>addBenchmarks</name>
      <anchorfile>classCorrade_1_1TestSuite_1_1Tester.html</anchorfile>
      <anchor>abd05bfd3a84e9bb0a280451df8e550e1</anchor>
      <arglist>(std::initializer_list&lt; void(Derived::*)()&gt; benchmarks, std::size_t batchCount, void(Derived::*setup)(), void(Derived::*teardown)(), BenchmarkType benchmarkType=BenchmarkType::Default)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>addCustomBenchmarks</name>
      <anchorfile>classCorrade_1_1TestSuite_1_1Tester.html</anchorfile>
      <anchor>abc2586121a787554a70113a654a5b5c3</anchor>
      <arglist>(std::initializer_list&lt; void(Derived::*)()&gt; benchmarks, std::size_t batchCount, void(Derived::*benchmarkBegin)(), std::uint64_t(Derived::*benchmarkEnd)(), BenchmarkUnits benchmarkUnits)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>addCustomBenchmarks</name>
      <anchorfile>classCorrade_1_1TestSuite_1_1Tester.html</anchorfile>
      <anchor>a41a55bef9a39c9672ccfa434ad0f101b</anchor>
      <arglist>(std::initializer_list&lt; void(Derived::*)()&gt; benchmarks, std::size_t batchCount, void(Derived::*setup)(), void(Derived::*teardown)(), void(Derived::*benchmarkBegin)(), std::uint64_t(Derived::*benchmarkEnd)(), BenchmarkUnits benchmarkUnits)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>addInstancedBenchmarks</name>
      <anchorfile>classCorrade_1_1TestSuite_1_1Tester.html</anchorfile>
      <anchor>a65cbff7b6714412f59ec842678850d5a</anchor>
      <arglist>(std::initializer_list&lt; void(Derived::*)()&gt; benchmarks, std::size_t batchCount, std::size_t instanceCount, BenchmarkType benchmarkType=BenchmarkType::Default)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>addInstancedBenchmarks</name>
      <anchorfile>classCorrade_1_1TestSuite_1_1Tester.html</anchorfile>
      <anchor>a5e420c73f8909475bcc4b036a04bb57e</anchor>
      <arglist>(std::initializer_list&lt; void(Derived::*)()&gt; benchmarks, std::size_t batchCount, std::size_t instanceCount, void(Derived::*setup)(), void(Derived::*teardown)(), BenchmarkType benchmarkType=BenchmarkType::Default)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>addCustomInstancedBenchmarks</name>
      <anchorfile>classCorrade_1_1TestSuite_1_1Tester.html</anchorfile>
      <anchor>ad97e1564731958ec41099b7728cac53e</anchor>
      <arglist>(std::initializer_list&lt; void(Derived::*)()&gt; benchmarks, std::size_t batchCount, std::size_t instanceCount, void(Derived::*benchmarkBegin)(), std::uint64_t(Derived::*benchmarkEnd)(), BenchmarkUnits benchmarkUnits)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>addCustomInstancedBenchmarks</name>
      <anchorfile>classCorrade_1_1TestSuite_1_1Tester.html</anchorfile>
      <anchor>a75eeb5cd700bb72eccd27d7159656236</anchor>
      <arglist>(std::initializer_list&lt; void(Derived::*)()&gt; benchmarks, std::size_t batchCount, std::size_t instanceCount, void(Derived::*setup)(), void(Derived::*teardown)(), void(Derived::*benchmarkBegin)(), std::uint64_t(Derived::*benchmarkEnd)(), BenchmarkUnits benchmarkUnits)</arglist>
    </member>
    <member kind="function">
      <type>std::size_t</type>
      <name>testCaseId</name>
      <anchorfile>classCorrade_1_1TestSuite_1_1Tester.html</anchorfile>
      <anchor>a11ead46b910fa99b9cba375f8339c6d3</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>std::size_t</type>
      <name>testCaseInstanceId</name>
      <anchorfile>classCorrade_1_1TestSuite_1_1Tester.html</anchorfile>
      <anchor>a6e4e68d5e55bdb862488688c964b7bc2</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>std::size_t</type>
      <name>testCaseRepeatId</name>
      <anchorfile>classCorrade_1_1TestSuite_1_1Tester.html</anchorfile>
      <anchor>a35501689a22896b7d08e3cb40dceffa4</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setTestName</name>
      <anchorfile>classCorrade_1_1TestSuite_1_1Tester.html</anchorfile>
      <anchor>a5df4a9a998c838fac32a592764dc1dc0</anchor>
      <arglist>(const std::string &amp;name)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setTestName</name>
      <anchorfile>classCorrade_1_1TestSuite_1_1Tester.html</anchorfile>
      <anchor>af2d00f73187b4016f8612f9668ee60e1</anchor>
      <arglist>(std::string &amp;&amp;name)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setTestName</name>
      <anchorfile>classCorrade_1_1TestSuite_1_1Tester.html</anchorfile>
      <anchor>af5624f0e668567ba6eb98e1e31d8648e</anchor>
      <arglist>(const char *name)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setTestCaseName</name>
      <anchorfile>classCorrade_1_1TestSuite_1_1Tester.html</anchorfile>
      <anchor>a2137bdada941b228a60c3905f208c5ee</anchor>
      <arglist>(const std::string &amp;name)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setTestCaseName</name>
      <anchorfile>classCorrade_1_1TestSuite_1_1Tester.html</anchorfile>
      <anchor>a76705bdce162388219938a6eea066671</anchor>
      <arglist>(std::string &amp;&amp;name)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setTestCaseName</name>
      <anchorfile>classCorrade_1_1TestSuite_1_1Tester.html</anchorfile>
      <anchor>ab988bb2fbfbb995615b16a4176e3c06e</anchor>
      <arglist>(const char *name)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setTestCaseTemplateName</name>
      <anchorfile>classCorrade_1_1TestSuite_1_1Tester.html</anchorfile>
      <anchor>a60d7e4cdf47970bc997028f4b80aa526</anchor>
      <arglist>(const std::string &amp;name)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setTestCaseTemplateName</name>
      <anchorfile>classCorrade_1_1TestSuite_1_1Tester.html</anchorfile>
      <anchor>aec108c5b7685c8c98f1f51ff26686c55</anchor>
      <arglist>(std::string &amp;&amp;name)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setTestCaseTemplateName</name>
      <anchorfile>classCorrade_1_1TestSuite_1_1Tester.html</anchorfile>
      <anchor>adeef46c36366d169cb69f685837ce4e5</anchor>
      <arglist>(const char *name)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setTestCaseDescription</name>
      <anchorfile>classCorrade_1_1TestSuite_1_1Tester.html</anchorfile>
      <anchor>a063f0a096ef06bdd093fb701ade5a794</anchor>
      <arglist>(const std::string &amp;description)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setTestCaseDescription</name>
      <anchorfile>classCorrade_1_1TestSuite_1_1Tester.html</anchorfile>
      <anchor>adb4bbf7a4d372490ae939ef32deade9d</anchor>
      <arglist>(std::string &amp;&amp;description)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setTestCaseDescription</name>
      <anchorfile>classCorrade_1_1TestSuite_1_1Tester.html</anchorfile>
      <anchor>afe7e87c77910ed3c25c7de2fdd863526</anchor>
      <arglist>(const char *description)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setBenchmarkName</name>
      <anchorfile>classCorrade_1_1TestSuite_1_1Tester.html</anchorfile>
      <anchor>ad5f846fd0c8a95aaf04179e69a0ddb32</anchor>
      <arglist>(const std::string &amp;name)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setBenchmarkName</name>
      <anchorfile>classCorrade_1_1TestSuite_1_1Tester.html</anchorfile>
      <anchor>a7f207ea485776874764b8f32f552d664</anchor>
      <arglist>(std::string &amp;&amp;name)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setBenchmarkName</name>
      <anchorfile>classCorrade_1_1TestSuite_1_1Tester.html</anchorfile>
      <anchor>abff854b31a3362cb56ebb54c22459a27</anchor>
      <arglist>(const char *name)</arglist>
    </member>
    <docanchor file="classCorrade_1_1TestSuite_1_1Tester" title="Basic testing workflow">TestSuite-Tester-basic</docanchor>
    <docanchor file="classCorrade_1_1TestSuite_1_1Tester" title="Instanced tests">TestSuite-Tester-instanced</docanchor>
    <docanchor file="classCorrade_1_1TestSuite_1_1Tester" title="Repeated tests">TestSuite-Tester-repeated</docanchor>
    <docanchor file="classCorrade_1_1TestSuite_1_1Tester" title="Saving diagnostic files">TestSuite-Tester-save-diagnostic</docanchor>
    <docanchor file="classCorrade_1_1TestSuite_1_1Tester" title="Benchmarks">TestSuite-Tester-benchmark</docanchor>
    <docanchor file="classCorrade_1_1TestSuite_1_1Tester" title="Custom benchmarks">TestSuite-Tester-benchmark-custom</docanchor>
    <docanchor file="classCorrade_1_1TestSuite_1_1Tester" title="Specifying setup/teardown routines">TestSuite-Tester-setup-teardown</docanchor>
    <docanchor file="classCorrade_1_1TestSuite_1_1Tester" title="Command-line options">TestSuite-Tester-command-line</docanchor>
    <docanchor file="classCorrade_1_1TestSuite_1_1Tester" title="Compiling and running tests">TestSuite-Tester-running</docanchor>
    <docanchor file="classCorrade_1_1TestSuite_1_1Tester" title="Using CMake">TestSuite-Tester-running-cmake</docanchor>
    <docanchor file="classCorrade_1_1TestSuite_1_1Tester" title="Manually running the tests on Android">TestSuite-Tester-running-android</docanchor>
    <docanchor file="classCorrade_1_1TestSuite_1_1Tester" title="Manually running the tests on Emscripten">TestSuite-Tester-running-emscripten</docanchor>
    <docanchor file="classCorrade_1_1TestSuite_1_1Tester" title="Running Emscripten tests in a browser">TestSuite-Tester-running-emscripten-browser</docanchor>
  </compound>
  <compound kind="class">
    <name>Corrade::TestSuite::Tester::TesterConfiguration</name>
    <filename>classCorrade_1_1TestSuite_1_1Tester_1_1TesterConfiguration.html</filename>
    <member kind="function">
      <type></type>
      <name>TesterConfiguration</name>
      <anchorfile>classCorrade_1_1TestSuite_1_1Tester_1_1TesterConfiguration.html</anchorfile>
      <anchor>aacb587778eaed8e4f47f04808c6a2511</anchor>
      <arglist>(const TesterConfiguration &amp;)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>TesterConfiguration</name>
      <anchorfile>classCorrade_1_1TestSuite_1_1Tester_1_1TesterConfiguration.html</anchorfile>
      <anchor>a9dbda656012b28c87b92fe4100751123</anchor>
      <arglist>(TesterConfiguration &amp;&amp;) noexcept</arglist>
    </member>
    <member kind="function">
      <type>TesterConfiguration &amp;</type>
      <name>operator=</name>
      <anchorfile>classCorrade_1_1TestSuite_1_1Tester_1_1TesterConfiguration.html</anchorfile>
      <anchor>a640bdcf559a28c433812ba491a04c65c</anchor>
      <arglist>(const TesterConfiguration &amp;)</arglist>
    </member>
    <member kind="function">
      <type>TesterConfiguration &amp;</type>
      <name>operator=</name>
      <anchorfile>classCorrade_1_1TestSuite_1_1Tester_1_1TesterConfiguration.html</anchorfile>
      <anchor>a64b93e0e6da90aa780c467dc5c3646c7</anchor>
      <arglist>(TesterConfiguration &amp;&amp;) noexcept</arglist>
    </member>
    <member kind="function">
      <type>Containers::ArrayView&lt; const std::string &gt;</type>
      <name>skippedArgumentPrefixes</name>
      <anchorfile>classCorrade_1_1TestSuite_1_1Tester_1_1TesterConfiguration.html</anchorfile>
      <anchor>a20e0d1c46609a56bc8273f6c4767ff5d</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>TesterConfiguration &amp;</type>
      <name>setSkippedArgumentPrefixes</name>
      <anchorfile>classCorrade_1_1TestSuite_1_1Tester_1_1TesterConfiguration.html</anchorfile>
      <anchor>aeac00c662bb910b46a248480878cad03</anchor>
      <arglist>(std::initializer_list&lt; std::string &gt; prefixes)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>Corrade::Utility::AbstractHash</name>
    <filename>classCorrade_1_1Utility_1_1AbstractHash.html</filename>
    <templarg>digestSize</templarg>
    <member kind="enumvalue">
      <name>DigestSize</name>
      <anchorfile>classCorrade_1_1Utility_1_1AbstractHash.html</anchorfile>
      <anchor>a44bc30cd5b5469847d54604642376d08a1845bcbe98f41a624c92ed7e797a14f6</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>HashDigest&lt; digestSize &gt;</type>
      <name>Digest</name>
      <anchorfile>classCorrade_1_1Utility_1_1AbstractHash.html</anchorfile>
      <anchor>a44db12441a55b94fdead99c880fd0a4e</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DigestSize</name>
      <anchorfile>classCorrade_1_1Utility_1_1AbstractHash.html</anchorfile>
      <anchor>a44bc30cd5b5469847d54604642376d08a1845bcbe98f41a624c92ed7e797a14f6</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>Corrade::Utility::AndroidLogStreamBuffer</name>
    <filename>classCorrade_1_1Utility_1_1AndroidLogStreamBuffer.html</filename>
    <base>std::stringbuf</base>
    <member kind="enumeration">
      <type></type>
      <name>LogPriority</name>
      <anchorfile>classCorrade_1_1Utility_1_1AndroidLogStreamBuffer.html</anchorfile>
      <anchor>a18ae87be1aa3d05f590a8ea2f26d538b</anchor>
      <arglist></arglist>
      <enumvalue file="classCorrade_1_1Utility_1_1AndroidLogStreamBuffer.html" anchor="a18ae87be1aa3d05f590a8ea2f26d538bad4a9fa383ab700c5bdd6f31cf7df0faf">Verbose</enumvalue>
      <enumvalue file="classCorrade_1_1Utility_1_1AndroidLogStreamBuffer.html" anchor="a18ae87be1aa3d05f590a8ea2f26d538baa603905470e2a5b8c13e96b579ef0dba">Debug</enumvalue>
      <enumvalue file="classCorrade_1_1Utility_1_1AndroidLogStreamBuffer.html" anchor="a18ae87be1aa3d05f590a8ea2f26d538ba4059b0251f66a18cb56f544728796875">Info</enumvalue>
      <enumvalue file="classCorrade_1_1Utility_1_1AndroidLogStreamBuffer.html" anchor="a18ae87be1aa3d05f590a8ea2f26d538ba0eaadb4fcb48a0a0ed7bc9868be9fbaa">Warning</enumvalue>
      <enumvalue file="classCorrade_1_1Utility_1_1AndroidLogStreamBuffer.html" anchor="a18ae87be1aa3d05f590a8ea2f26d538ba902b0d55fddef6f8d651fe1035b7d4bd">Error</enumvalue>
      <enumvalue file="classCorrade_1_1Utility_1_1AndroidLogStreamBuffer.html" anchor="a18ae87be1aa3d05f590a8ea2f26d538ba882384ec38ce8d9582b57e70861730e4">Fatal</enumvalue>
    </member>
    <member kind="function">
      <type></type>
      <name>AndroidLogStreamBuffer</name>
      <anchorfile>classCorrade_1_1Utility_1_1AndroidLogStreamBuffer.html</anchorfile>
      <anchor>ab631a4d21e449fa7fc4ba337f6663b97</anchor>
      <arglist>(LogPriority priority, std::string tag)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>Corrade::Utility::Arguments</name>
    <filename>classCorrade_1_1Utility_1_1Arguments.html</filename>
    <member kind="enumeration">
      <type></type>
      <name>Flag</name>
      <anchorfile>classCorrade_1_1Utility_1_1Arguments.html</anchorfile>
      <anchor>a82f8fcf588caf0b2e41b980ef5fd6481</anchor>
      <arglist></arglist>
      <enumvalue file="classCorrade_1_1Utility_1_1Arguments.html" anchor="a82f8fcf588caf0b2e41b980ef5fd6481a700850787976272768547cad8f9973af">IgnoreUnknownOptions</enumvalue>
    </member>
    <member kind="typedef">
      <type>Containers::EnumSet&lt; Flag &gt;</type>
      <name>Flags</name>
      <anchorfile>classCorrade_1_1Utility_1_1Arguments.html</anchorfile>
      <anchor>a5b0c6984c66c3ee9d02fce798279dc0a</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Arguments</name>
      <anchorfile>classCorrade_1_1Utility_1_1Arguments.html</anchorfile>
      <anchor>ac83f6669b1b26cf2ba722d2e49a55a7d</anchor>
      <arglist>(Flags flags={})</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Arguments</name>
      <anchorfile>classCorrade_1_1Utility_1_1Arguments.html</anchorfile>
      <anchor>a1ac43b5236e3889c49c758da5ed055b7</anchor>
      <arglist>(const std::string &amp;prefix, Flags flags={})</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Arguments</name>
      <anchorfile>classCorrade_1_1Utility_1_1Arguments.html</anchorfile>
      <anchor>ae79d16b05e600cb47aba5514351f01c4</anchor>
      <arglist>(const Arguments &amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Arguments</name>
      <anchorfile>classCorrade_1_1Utility_1_1Arguments.html</anchorfile>
      <anchor>ac1cf07c2e2b675d996170b06e754bbde</anchor>
      <arglist>(Arguments &amp;&amp;other) noexcept</arglist>
    </member>
    <member kind="function">
      <type>Arguments &amp;</type>
      <name>operator=</name>
      <anchorfile>classCorrade_1_1Utility_1_1Arguments.html</anchorfile>
      <anchor>a3311137d89812d8b488741d3c63bdcaa</anchor>
      <arglist>(const Arguments &amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type>Arguments &amp;</type>
      <name>operator=</name>
      <anchorfile>classCorrade_1_1Utility_1_1Arguments.html</anchorfile>
      <anchor>ae61fb3d170d2a6c287508f998f6d37c8</anchor>
      <arglist>(Arguments &amp;&amp;other) noexcept</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>prefix</name>
      <anchorfile>classCorrade_1_1Utility_1_1Arguments.html</anchorfile>
      <anchor>a7d708f9420e66d17823221fda9ee7ddf</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>isParsed</name>
      <anchorfile>classCorrade_1_1Utility_1_1Arguments.html</anchorfile>
      <anchor>ac2f9fe31a2518370529778d7d652ea5e</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>Arguments &amp;</type>
      <name>addArgument</name>
      <anchorfile>classCorrade_1_1Utility_1_1Arguments.html</anchorfile>
      <anchor>af1aa8eb7d9af745fcaa9042b8c915d96</anchor>
      <arglist>(std::string key)</arglist>
    </member>
    <member kind="function">
      <type>Arguments &amp;</type>
      <name>addNamedArgument</name>
      <anchorfile>classCorrade_1_1Utility_1_1Arguments.html</anchorfile>
      <anchor>aa06be9626ed02314671e918867eed2b5</anchor>
      <arglist>(char shortKey, std::string key)</arglist>
    </member>
    <member kind="function">
      <type>Arguments &amp;</type>
      <name>addNamedArgument</name>
      <anchorfile>classCorrade_1_1Utility_1_1Arguments.html</anchorfile>
      <anchor>a2bf98b4f9617759235e19e5486cc8b0d</anchor>
      <arglist>(std::string key)</arglist>
    </member>
    <member kind="function">
      <type>Arguments &amp;</type>
      <name>addOption</name>
      <anchorfile>classCorrade_1_1Utility_1_1Arguments.html</anchorfile>
      <anchor>a5ef44d52ebeaa2eea906088ffe6e1537</anchor>
      <arglist>(char shortKey, std::string key, std::string defaultValue=std::string())</arglist>
    </member>
    <member kind="function">
      <type>Arguments &amp;</type>
      <name>addOption</name>
      <anchorfile>classCorrade_1_1Utility_1_1Arguments.html</anchorfile>
      <anchor>abbe0aa87021d58e15fc2bf2fb34e065b</anchor>
      <arglist>(std::string key, std::string defaultValue=std::string())</arglist>
    </member>
    <member kind="function">
      <type>Arguments &amp;</type>
      <name>addBooleanOption</name>
      <anchorfile>classCorrade_1_1Utility_1_1Arguments.html</anchorfile>
      <anchor>a88e288320ad981adbd3410b3b5f7b355</anchor>
      <arglist>(char shortKey, std::string key)</arglist>
    </member>
    <member kind="function">
      <type>Arguments &amp;</type>
      <name>addBooleanOption</name>
      <anchorfile>classCorrade_1_1Utility_1_1Arguments.html</anchorfile>
      <anchor>aa78f024b302dd98f0133113f64029f96</anchor>
      <arglist>(std::string key)</arglist>
    </member>
    <member kind="function">
      <type>Arguments &amp;</type>
      <name>addFinalOptionalArgument</name>
      <anchorfile>classCorrade_1_1Utility_1_1Arguments.html</anchorfile>
      <anchor>a2ea130798e6d9f236300989414068598</anchor>
      <arglist>(std::string key, std::string defaultValue=std::string())</arglist>
    </member>
    <member kind="function">
      <type>Arguments &amp;</type>
      <name>addSkippedPrefix</name>
      <anchorfile>classCorrade_1_1Utility_1_1Arguments.html</anchorfile>
      <anchor>a2b05c808d57135c37d6229fc250af38c</anchor>
      <arglist>(std::string prefix, std::string help={})</arglist>
    </member>
    <member kind="function">
      <type>Arguments &amp;</type>
      <name>setFromEnvironment</name>
      <anchorfile>classCorrade_1_1Utility_1_1Arguments.html</anchorfile>
      <anchor>a0bbc9455c3615ba42f6c1d4cb52f80d5</anchor>
      <arglist>(const std::string &amp;key, std::string environmentVariable)</arglist>
    </member>
    <member kind="function">
      <type>Arguments &amp;</type>
      <name>setFromEnvironment</name>
      <anchorfile>classCorrade_1_1Utility_1_1Arguments.html</anchorfile>
      <anchor>a2da1065d616c7d5f6a32c8eb576c6f79</anchor>
      <arglist>(const std::string &amp;key)</arglist>
    </member>
    <member kind="function">
      <type>Arguments &amp;</type>
      <name>setCommand</name>
      <anchorfile>classCorrade_1_1Utility_1_1Arguments.html</anchorfile>
      <anchor>a5d53692ba045cf61283d8c882dbb78c3</anchor>
      <arglist>(std::string name)</arglist>
    </member>
    <member kind="function">
      <type>Arguments &amp;</type>
      <name>setGlobalHelp</name>
      <anchorfile>classCorrade_1_1Utility_1_1Arguments.html</anchorfile>
      <anchor>a1e7e1b5e18a92764fa1167d414030a65</anchor>
      <arglist>(std::string help)</arglist>
    </member>
    <member kind="function">
      <type>Arguments &amp;</type>
      <name>setHelp</name>
      <anchorfile>classCorrade_1_1Utility_1_1Arguments.html</anchorfile>
      <anchor>ae2e7b65d81f67bf99efb7bc9b530417b</anchor>
      <arglist>(std::string help)</arglist>
    </member>
    <member kind="function">
      <type>Arguments &amp;</type>
      <name>setHelp</name>
      <anchorfile>classCorrade_1_1Utility_1_1Arguments.html</anchorfile>
      <anchor>a84a3e0606d26057ae6a71e5b645572b9</anchor>
      <arglist>(const std::string &amp;key, std::string help, std::string helpKey={})</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>parse</name>
      <anchorfile>classCorrade_1_1Utility_1_1Arguments.html</anchorfile>
      <anchor>adb04863348b937593426df10f43d99d8</anchor>
      <arglist>(int argc, const char **argv)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>parse</name>
      <anchorfile>classCorrade_1_1Utility_1_1Arguments.html</anchorfile>
      <anchor>a908e3c2f42fa3b659e15ff3f8e5ddd9d</anchor>
      <arglist>(int argc, char **argv)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>parse</name>
      <anchorfile>classCorrade_1_1Utility_1_1Arguments.html</anchorfile>
      <anchor>a6f1911c2bc595f66f17f9f5675ca1db4</anchor>
      <arglist>(int argc, std::nullptr_t argv)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>tryParse</name>
      <anchorfile>classCorrade_1_1Utility_1_1Arguments.html</anchorfile>
      <anchor>a5c11b3c6850c8c90602d9ffad0478f24</anchor>
      <arglist>(int argc, const char **argv)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>tryParse</name>
      <anchorfile>classCorrade_1_1Utility_1_1Arguments.html</anchorfile>
      <anchor>a067f02f2a75c2f71b6e05c35345a073f</anchor>
      <arglist>(int argc, char **argv)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>tryParse</name>
      <anchorfile>classCorrade_1_1Utility_1_1Arguments.html</anchorfile>
      <anchor>a0c2bc66a2ca1a24d905019c48a644dd9</anchor>
      <arglist>(int argc, std::nullptr_t argv)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>usage</name>
      <anchorfile>classCorrade_1_1Utility_1_1Arguments.html</anchorfile>
      <anchor>aa0c776cec0bb73620710355b8f9f3a33</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>help</name>
      <anchorfile>classCorrade_1_1Utility_1_1Arguments.html</anchorfile>
      <anchor>ab3ae8e4ded6d537b7c2e1c9ffe0b8487</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>T</type>
      <name>value</name>
      <anchorfile>classCorrade_1_1Utility_1_1Arguments.html</anchorfile>
      <anchor>a52f8a7ecef8af0073c17f82845e9b8d1</anchor>
      <arglist>(const std::string &amp;key, ConfigurationValueFlags flags={}) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>isSet</name>
      <anchorfile>classCorrade_1_1Utility_1_1Arguments.html</anchorfile>
      <anchor>aeb72baaf9199afec94d43d3a836091ae</anchor>
      <arglist>(const std::string &amp;key) const</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static std::vector&lt; std::string &gt;</type>
      <name>environment</name>
      <anchorfile>classCorrade_1_1Utility_1_1Arguments.html</anchorfile>
      <anchor>a2310c3fb9f928461b621babd7a2c7e29</anchor>
      <arglist>()</arglist>
    </member>
    <docanchor file="classCorrade_1_1Utility_1_1Arguments" title="Example usage">Utility-Arguments-usage</docanchor>
    <docanchor file="classCorrade_1_1Utility_1_1Arguments" title="Delegating arguments to different parts of the application">Utility-Arguments-delegating</docanchor>
  </compound>
  <compound kind="class">
    <name>Corrade::Utility::Configuration</name>
    <filename>classCorrade_1_1Utility_1_1Configuration.html</filename>
    <base>Corrade::Utility::ConfigurationGroup</base>
    <member kind="enumeration">
      <type></type>
      <name>Flag</name>
      <anchorfile>classCorrade_1_1Utility_1_1Configuration.html</anchorfile>
      <anchor>a5d368fa6993f223b4c3fa25332921e86</anchor>
      <arglist></arglist>
      <enumvalue file="classCorrade_1_1Utility_1_1Configuration.html" anchor="a5d368fa6993f223b4c3fa25332921e86afaebb611045e49cc0dfa08935d38be89">PreserveBom</enumvalue>
      <enumvalue file="classCorrade_1_1Utility_1_1Configuration.html" anchor="a5d368fa6993f223b4c3fa25332921e86a03cf6a68c2fac661330a1ad2cac38b90">ForceUnixEol</enumvalue>
      <enumvalue file="classCorrade_1_1Utility_1_1Configuration.html" anchor="a5d368fa6993f223b4c3fa25332921e86a6f417e6c5ec8c6620ad71177b1e4d3aa">ForceWindowsEol</enumvalue>
      <enumvalue file="classCorrade_1_1Utility_1_1Configuration.html" anchor="a5d368fa6993f223b4c3fa25332921e86aa8156810bfee2bd2b44765b9e91db3bd">Truncate</enumvalue>
      <enumvalue file="classCorrade_1_1Utility_1_1Configuration.html" anchor="a5d368fa6993f223b4c3fa25332921e86ae076500ec82f42a92c1291f3c2ebdb72">SkipComments</enumvalue>
      <enumvalue file="classCorrade_1_1Utility_1_1Configuration.html" anchor="a5d368fa6993f223b4c3fa25332921e86a131fb182a881796e7606ed6da27f1197">ReadOnly</enumvalue>
    </member>
    <member kind="typedef">
      <type>Containers::EnumSet&lt; Flag &gt;</type>
      <name>Flags</name>
      <anchorfile>classCorrade_1_1Utility_1_1Configuration.html</anchorfile>
      <anchor>a0b3f492e864ee97228d5170d38a31933</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Configuration</name>
      <anchorfile>classCorrade_1_1Utility_1_1Configuration.html</anchorfile>
      <anchor>abe8c1af9685dc46bce30ee46203f895c</anchor>
      <arglist>(Flags flags=Flags())</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Configuration</name>
      <anchorfile>classCorrade_1_1Utility_1_1Configuration.html</anchorfile>
      <anchor>a5aa6826a44ae881fdd9a3b7ff4b57535</anchor>
      <arglist>(const std::string &amp;filename, Flags flags=Flags())</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Configuration</name>
      <anchorfile>classCorrade_1_1Utility_1_1Configuration.html</anchorfile>
      <anchor>aa9808d44a604e4a455975e3ebfd6bf37</anchor>
      <arglist>(std::istream &amp;in, Flags flags=Flags())</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Configuration</name>
      <anchorfile>classCorrade_1_1Utility_1_1Configuration.html</anchorfile>
      <anchor>a7ac737721e13c9303696985e2a8672e1</anchor>
      <arglist>(const Configuration &amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Configuration</name>
      <anchorfile>classCorrade_1_1Utility_1_1Configuration.html</anchorfile>
      <anchor>a1258e56c643add518646e32edaf99892</anchor>
      <arglist>(Configuration &amp;&amp;other)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~Configuration</name>
      <anchorfile>classCorrade_1_1Utility_1_1Configuration.html</anchorfile>
      <anchor>ae48021eb27720034fdebaa9dd1adad27</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>Configuration &amp;</type>
      <name>operator=</name>
      <anchorfile>classCorrade_1_1Utility_1_1Configuration.html</anchorfile>
      <anchor>ab54d47655023263934e52520ba00ba4a</anchor>
      <arglist>(const Configuration &amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type>Configuration &amp;</type>
      <name>operator=</name>
      <anchorfile>classCorrade_1_1Utility_1_1Configuration.html</anchorfile>
      <anchor>a755a0d60b854f2a78e5f6eccd9fe452b</anchor>
      <arglist>(Configuration &amp;&amp;other)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>filename</name>
      <anchorfile>classCorrade_1_1Utility_1_1Configuration.html</anchorfile>
      <anchor>a9d3d905aafa6132053c1ed665c323fa8</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setFilename</name>
      <anchorfile>classCorrade_1_1Utility_1_1Configuration.html</anchorfile>
      <anchor>aa8745aca008e75311acbb3778ba81fd0</anchor>
      <arglist>(std::string filename)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>isValid</name>
      <anchorfile>classCorrade_1_1Utility_1_1Configuration.html</anchorfile>
      <anchor>a339906c024f64b895238bf099b5ed241</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>save</name>
      <anchorfile>classCorrade_1_1Utility_1_1Configuration.html</anchorfile>
      <anchor>a4e6f38caab000dd04321001f4445d1ff</anchor>
      <arglist>(const std::string &amp;filename)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>save</name>
      <anchorfile>classCorrade_1_1Utility_1_1Configuration.html</anchorfile>
      <anchor>abc47d18cdd33e5c928f2aa33dd6b09fd</anchor>
      <arglist>(std::ostream &amp;out)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>save</name>
      <anchorfile>classCorrade_1_1Utility_1_1Configuration.html</anchorfile>
      <anchor>ae6d938117a8afa18976fc9a481220f2e</anchor>
      <arglist>()</arglist>
    </member>
    <docanchor file="classCorrade_1_1Utility_1_1Configuration" title="File syntax and usage">Utility-Configuration-usage</docanchor>
    <docanchor file="classCorrade_1_1Utility_1_1Configuration" title="Whitespace and comments">Utility-Configuration-usage-whitespace</docanchor>
    <docanchor file="classCorrade_1_1Utility_1_1Configuration" title="Key/value pairs">Utility-Configuration-usage-values</docanchor>
    <docanchor file="classCorrade_1_1Utility_1_1Configuration" title="Value groups">Utility-Configuration-usage-groups</docanchor>
  </compound>
  <compound kind="class">
    <name>Corrade::Utility::ConfigurationGroup</name>
    <filename>classCorrade_1_1Utility_1_1ConfigurationGroup.html</filename>
    <member kind="function">
      <type></type>
      <name>ConfigurationGroup</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>a500d96ba04e6c01b95d9121cf4934f18</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>ConfigurationGroup</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>ab41d1f58bfcf9d6b7cb5727f87429e1e</anchor>
      <arglist>(const ConfigurationGroup &amp;other)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>ConfigurationGroup</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>a1ae4cecc4e8cf32c7e4d3aaa286bdc59</anchor>
      <arglist>(ConfigurationGroup &amp;&amp;other)</arglist>
    </member>
    <member kind="function">
      <type>ConfigurationGroup &amp;</type>
      <name>operator=</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>a291f0b9770236cc43b9ccbc17ea2b6f3</anchor>
      <arglist>(const ConfigurationGroup &amp;other)</arglist>
    </member>
    <member kind="function">
      <type>ConfigurationGroup &amp;</type>
      <name>operator=</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>a1172fdd7703f23820d7c92057e3b86e4</anchor>
      <arglist>(ConfigurationGroup &amp;&amp;other)</arglist>
    </member>
    <member kind="function">
      <type>Configuration *</type>
      <name>configuration</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>a6ac83e7385b39cd9a1c08aeb9a7201d8</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const Configuration *</type>
      <name>configuration</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>a0e50b5c57375c113cee93fbd4a1b645a</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>isEmpty</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>a5f33c8854891aae82b6f102b82744355</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>clear</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>a527a6fb19869f89e99ae052d1d26a5e5</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>hasGroups</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>a650866079ff8f98e317a14f4fd150a45</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>unsigned int</type>
      <name>groupCount</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>abdcb04c6c1db224dd7786e638606d8e9</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>hasGroup</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>ac2f358ab27af981ff68ea9292f59a020</anchor>
      <arglist>(const std::string &amp;name, unsigned int index=0) const</arglist>
    </member>
    <member kind="function">
      <type>unsigned int</type>
      <name>groupCount</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>a3eabf45c8bc4cfd9d807648c452f8ec5</anchor>
      <arglist>(const std::string &amp;name) const</arglist>
    </member>
    <member kind="function">
      <type>ConfigurationGroup *</type>
      <name>group</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>a2bc02705571e6eb3fee1077c9c61a573</anchor>
      <arglist>(const std::string &amp;name, unsigned int index=0)</arglist>
    </member>
    <member kind="function">
      <type>const ConfigurationGroup *</type>
      <name>group</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>a114fe116b7e0643bcebdab5f0cd47f47</anchor>
      <arglist>(const std::string &amp;name, unsigned int index=0) const</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; ConfigurationGroup * &gt;</type>
      <name>groups</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>aa5c8f92f48ac44d7d3d084622e6df6cc</anchor>
      <arglist>(const std::string &amp;name)</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; const ConfigurationGroup * &gt;</type>
      <name>groups</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>a362cc3e2239d816b0fe19d7f797bc9e1</anchor>
      <arglist>(const std::string &amp;name) const</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>addGroup</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>a283661713aa073739663b46f5c468e1c</anchor>
      <arglist>(const std::string &amp;name, ConfigurationGroup *group)</arglist>
    </member>
    <member kind="function">
      <type>ConfigurationGroup *</type>
      <name>addGroup</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>a0d97d4fd435ba78a1c8d96aca8019b75</anchor>
      <arglist>(const std::string &amp;name)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>removeGroup</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>a6eeed46dd76a629fd2693732c91d0040</anchor>
      <arglist>(const std::string &amp;name, unsigned int index=0)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>removeGroup</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>afe24d64f8c78513293e57e38c7d0727c</anchor>
      <arglist>(ConfigurationGroup *group)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>removeAllGroups</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>a84000627f3d7e0e07d0a6ffa09c2c636</anchor>
      <arglist>(const std::string &amp;name)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>hasValues</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>a58f76764f03de9b5762456236a8fcdd2</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>unsigned int</type>
      <name>valueCount</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>aad421cdb9c20c0af6edb4fcd74602ac8</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>hasValue</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>adf227126a78b3ff49a7afa93bcff2ad0</anchor>
      <arglist>(const std::string &amp;key, unsigned int index=0) const</arglist>
    </member>
    <member kind="function">
      <type>unsigned int</type>
      <name>valueCount</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>ac1f3413007d0239b9602c49bd998660c</anchor>
      <arglist>(const std::string &amp;key) const</arglist>
    </member>
    <member kind="function">
      <type>T</type>
      <name>value</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>aa201500e4da6274b621430d4d9aaa7bf</anchor>
      <arglist>(const std::string &amp;key, unsigned int index=0, ConfigurationValueFlags flags=ConfigurationValueFlags()) const</arglist>
    </member>
    <member kind="function">
      <type>T</type>
      <name>value</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>acd283e52686ac4ed50a397aea968c431</anchor>
      <arglist>(const std::string &amp;key, ConfigurationValueFlags flags) const</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; T &gt;</type>
      <name>values</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>af212fb2c4e59b7a1f1c1027329b5bd67</anchor>
      <arglist>(const std::string &amp;key, ConfigurationValueFlags flags=ConfigurationValueFlags()) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>setValue</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>aadca4a5b40bcd66d0837193af2c841e2</anchor>
      <arglist>(const std::string &amp;key, std::string value, unsigned int index=0, ConfigurationValueFlags flags=ConfigurationValueFlags())</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>setValue</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>a8d881c499196d832b7dd3581df25ab55</anchor>
      <arglist>(const std::string &amp;key, const char *value, unsigned int index=0, ConfigurationValueFlags flags=ConfigurationValueFlags())</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>setValue</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>a1be7095f3d51a0d6fe324545c62d569e</anchor>
      <arglist>(const std::string &amp;key, std::string value, ConfigurationValueFlags flags)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>setValue</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>a025de56172c2b6ff9640053b6a72eb00</anchor>
      <arglist>(const std::string &amp;key, const char *value, ConfigurationValueFlags flags)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>setValue</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>aa2988c7a715adb37b3d8938a08b9e006</anchor>
      <arglist>(const std::string &amp;key, const T &amp;value, unsigned int index=0, ConfigurationValueFlags flags=ConfigurationValueFlags())</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>setValue</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>abf56c0ee12977dd024ede9cfcd1fda41</anchor>
      <arglist>(const std::string &amp;key, const T &amp;value, ConfigurationValueFlags flags)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>addValue</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>abf66026df64ae2561040aaa75fdfbd31</anchor>
      <arglist>(std::string key, std::string value, ConfigurationValueFlags flags=ConfigurationValueFlags())</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>addValue</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>a6ba87151d0821e99d9e5038586ffed71</anchor>
      <arglist>(std::string key, const char *value, ConfigurationValueFlags flags=ConfigurationValueFlags())</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>addValue</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>aee6029a57056ff1e3a4400d8da424c48</anchor>
      <arglist>(std::string key, const T &amp;value, ConfigurationValueFlags flags=ConfigurationValueFlags())</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>removeValue</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>a1b7d7d5fbd18eaf10c3aafe4871509ad</anchor>
      <arglist>(const std::string &amp;key, unsigned int index=0)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>removeAllValues</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>a1654bb0a8323a813f380004a6fb57759</anchor>
      <arglist>(const std::string &amp;key)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>hasGroups</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>a650866079ff8f98e317a14f4fd150a45</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>unsigned int</type>
      <name>groupCount</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>abdcb04c6c1db224dd7786e638606d8e9</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>hasGroup</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>ac2f358ab27af981ff68ea9292f59a020</anchor>
      <arglist>(const std::string &amp;name, unsigned int index=0) const</arglist>
    </member>
    <member kind="function">
      <type>unsigned int</type>
      <name>groupCount</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>a3eabf45c8bc4cfd9d807648c452f8ec5</anchor>
      <arglist>(const std::string &amp;name) const</arglist>
    </member>
    <member kind="function">
      <type>ConfigurationGroup *</type>
      <name>group</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>a2bc02705571e6eb3fee1077c9c61a573</anchor>
      <arglist>(const std::string &amp;name, unsigned int index=0)</arglist>
    </member>
    <member kind="function">
      <type>const ConfigurationGroup *</type>
      <name>group</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>a114fe116b7e0643bcebdab5f0cd47f47</anchor>
      <arglist>(const std::string &amp;name, unsigned int index=0) const</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; ConfigurationGroup * &gt;</type>
      <name>groups</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>aa5c8f92f48ac44d7d3d084622e6df6cc</anchor>
      <arglist>(const std::string &amp;name)</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; const ConfigurationGroup * &gt;</type>
      <name>groups</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>a362cc3e2239d816b0fe19d7f797bc9e1</anchor>
      <arglist>(const std::string &amp;name) const</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>addGroup</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>a283661713aa073739663b46f5c468e1c</anchor>
      <arglist>(const std::string &amp;name, ConfigurationGroup *group)</arglist>
    </member>
    <member kind="function">
      <type>ConfigurationGroup *</type>
      <name>addGroup</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>a0d97d4fd435ba78a1c8d96aca8019b75</anchor>
      <arglist>(const std::string &amp;name)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>removeGroup</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>a6eeed46dd76a629fd2693732c91d0040</anchor>
      <arglist>(const std::string &amp;name, unsigned int index=0)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>removeGroup</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>afe24d64f8c78513293e57e38c7d0727c</anchor>
      <arglist>(ConfigurationGroup *group)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>removeAllGroups</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>a84000627f3d7e0e07d0a6ffa09c2c636</anchor>
      <arglist>(const std::string &amp;name)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>hasValues</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>a58f76764f03de9b5762456236a8fcdd2</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>unsigned int</type>
      <name>valueCount</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>aad421cdb9c20c0af6edb4fcd74602ac8</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>hasValue</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>adf227126a78b3ff49a7afa93bcff2ad0</anchor>
      <arglist>(const std::string &amp;key, unsigned int index=0) const</arglist>
    </member>
    <member kind="function">
      <type>unsigned int</type>
      <name>valueCount</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>ac1f3413007d0239b9602c49bd998660c</anchor>
      <arglist>(const std::string &amp;key) const</arglist>
    </member>
    <member kind="function">
      <type>T</type>
      <name>value</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>aa201500e4da6274b621430d4d9aaa7bf</anchor>
      <arglist>(const std::string &amp;key, unsigned int index=0, ConfigurationValueFlags flags=ConfigurationValueFlags()) const</arglist>
    </member>
    <member kind="function">
      <type>T</type>
      <name>value</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>acd283e52686ac4ed50a397aea968c431</anchor>
      <arglist>(const std::string &amp;key, ConfigurationValueFlags flags) const</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; T &gt;</type>
      <name>values</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>af212fb2c4e59b7a1f1c1027329b5bd67</anchor>
      <arglist>(const std::string &amp;key, ConfigurationValueFlags flags=ConfigurationValueFlags()) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>setValue</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>aadca4a5b40bcd66d0837193af2c841e2</anchor>
      <arglist>(const std::string &amp;key, std::string value, unsigned int index=0, ConfigurationValueFlags flags=ConfigurationValueFlags())</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>setValue</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>a8d881c499196d832b7dd3581df25ab55</anchor>
      <arglist>(const std::string &amp;key, const char *value, unsigned int index=0, ConfigurationValueFlags flags=ConfigurationValueFlags())</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>setValue</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>a1be7095f3d51a0d6fe324545c62d569e</anchor>
      <arglist>(const std::string &amp;key, std::string value, ConfigurationValueFlags flags)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>setValue</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>a025de56172c2b6ff9640053b6a72eb00</anchor>
      <arglist>(const std::string &amp;key, const char *value, ConfigurationValueFlags flags)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>setValue</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>aa2988c7a715adb37b3d8938a08b9e006</anchor>
      <arglist>(const std::string &amp;key, const T &amp;value, unsigned int index=0, ConfigurationValueFlags flags=ConfigurationValueFlags())</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>setValue</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>abf56c0ee12977dd024ede9cfcd1fda41</anchor>
      <arglist>(const std::string &amp;key, const T &amp;value, ConfigurationValueFlags flags)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>addValue</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>abf66026df64ae2561040aaa75fdfbd31</anchor>
      <arglist>(std::string key, std::string value, ConfigurationValueFlags flags=ConfigurationValueFlags())</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>addValue</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>a6ba87151d0821e99d9e5038586ffed71</anchor>
      <arglist>(std::string key, const char *value, ConfigurationValueFlags flags=ConfigurationValueFlags())</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>addValue</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>aee6029a57056ff1e3a4400d8da424c48</anchor>
      <arglist>(std::string key, const T &amp;value, ConfigurationValueFlags flags=ConfigurationValueFlags())</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>removeValue</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>a1b7d7d5fbd18eaf10c3aafe4871509ad</anchor>
      <arglist>(const std::string &amp;key, unsigned int index=0)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>removeAllValues</name>
      <anchorfile>classCorrade_1_1Utility_1_1ConfigurationGroup.html</anchorfile>
      <anchor>a1654bb0a8323a813f380004a6fb57759</anchor>
      <arglist>(const std::string &amp;key)</arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>Corrade::Utility::ConfigurationValue</name>
    <filename>structCorrade_1_1Utility_1_1ConfigurationValue.html</filename>
    <templarg></templarg>
    <member kind="function" static="yes">
      <type>static std::string</type>
      <name>toString</name>
      <anchorfile>structCorrade_1_1Utility_1_1ConfigurationValue.html</anchorfile>
      <anchor>ab639d070423f77173baa8ada58975ffa</anchor>
      <arglist>(const T &amp;value, ConfigurationValueFlags flags)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static T</type>
      <name>fromString</name>
      <anchorfile>structCorrade_1_1Utility_1_1ConfigurationValue.html</anchorfile>
      <anchor>a5e3bd19c82a34fb97bfddc94404f932a</anchor>
      <arglist>(const std::string &amp;stringValue, ConfigurationValueFlags flags)</arglist>
    </member>
    <docanchor file="structCorrade_1_1Utility_1_1ConfigurationValue" title="Example: custom structure">Utility-ConfigurationValue-example</docanchor>
  </compound>
  <compound kind="struct">
    <name>Corrade::Utility::ConfigurationValue&lt; bool &gt;</name>
    <filename>structCorrade_1_1Utility_1_1ConfigurationValue_3_01bool_01_4.html</filename>
  </compound>
  <compound kind="struct">
    <name>Corrade::Utility::ConfigurationValue&lt; char32_t &gt;</name>
    <filename>structCorrade_1_1Utility_1_1ConfigurationValue_3_01char32__t_01_4.html</filename>
  </compound>
  <compound kind="struct">
    <name>Corrade::Utility::ConfigurationValue&lt; double &gt;</name>
    <filename>structCorrade_1_1Utility_1_1ConfigurationValue_3_01double_01_4.html</filename>
  </compound>
  <compound kind="struct">
    <name>Corrade::Utility::ConfigurationValue&lt; float &gt;</name>
    <filename>structCorrade_1_1Utility_1_1ConfigurationValue_3_01float_01_4.html</filename>
  </compound>
  <compound kind="struct">
    <name>Corrade::Utility::ConfigurationValue&lt; int &gt;</name>
    <filename>structCorrade_1_1Utility_1_1ConfigurationValue_3_01int_01_4.html</filename>
  </compound>
  <compound kind="struct">
    <name>Corrade::Utility::ConfigurationValue&lt; long &gt;</name>
    <filename>structCorrade_1_1Utility_1_1ConfigurationValue_3_01long_01_4.html</filename>
  </compound>
  <compound kind="struct">
    <name>Corrade::Utility::ConfigurationValue&lt; long double &gt;</name>
    <filename>structCorrade_1_1Utility_1_1ConfigurationValue_3_01long_01double_01_4.html</filename>
  </compound>
  <compound kind="struct">
    <name>Corrade::Utility::ConfigurationValue&lt; long long &gt;</name>
    <filename>structCorrade_1_1Utility_1_1ConfigurationValue_3_01long_01long_01_4.html</filename>
  </compound>
  <compound kind="struct">
    <name>Corrade::Utility::ConfigurationValue&lt; short &gt;</name>
    <filename>structCorrade_1_1Utility_1_1ConfigurationValue_3_01short_01_4.html</filename>
  </compound>
  <compound kind="struct">
    <name>Corrade::Utility::ConfigurationValue&lt; std::string &gt;</name>
    <filename>structCorrade_1_1Utility_1_1ConfigurationValue_3_01std_1_1string_01_4.html</filename>
  </compound>
  <compound kind="struct">
    <name>Corrade::Utility::ConfigurationValue&lt; unsigned int &gt;</name>
    <filename>structCorrade_1_1Utility_1_1ConfigurationValue_3_01unsigned_01int_01_4.html</filename>
  </compound>
  <compound kind="struct">
    <name>Corrade::Utility::ConfigurationValue&lt; unsigned long &gt;</name>
    <filename>structCorrade_1_1Utility_1_1ConfigurationValue_3_01unsigned_01long_01_4.html</filename>
  </compound>
  <compound kind="struct">
    <name>Corrade::Utility::ConfigurationValue&lt; unsigned long long &gt;</name>
    <filename>structCorrade_1_1Utility_1_1ConfigurationValue_3_01unsigned_01long_01long_01_4.html</filename>
  </compound>
  <compound kind="struct">
    <name>Corrade::Utility::ConfigurationValue&lt; unsigned short &gt;</name>
    <filename>structCorrade_1_1Utility_1_1ConfigurationValue_3_01unsigned_01short_01_4.html</filename>
  </compound>
  <compound kind="class">
    <name>Corrade::Utility::Debug</name>
    <filename>classCorrade_1_1Utility_1_1Debug.html</filename>
    <member kind="enumeration">
      <type></type>
      <name>Flag</name>
      <anchorfile>classCorrade_1_1Utility_1_1Debug.html</anchorfile>
      <anchor>aa346e932b5141006c49777ddacfaf600</anchor>
      <arglist></arglist>
      <enumvalue file="classCorrade_1_1Utility_1_1Debug.html" anchor="aa346e932b5141006c49777ddacfaf600af29e1bcd00663e0b5d5a74d31baa9e96">NoNewlineAtTheEnd</enumvalue>
      <enumvalue file="classCorrade_1_1Utility_1_1Debug.html" anchor="aa346e932b5141006c49777ddacfaf600ada953bf4ea2f8e90228cd77eb880389f">DisableColors</enumvalue>
      <enumvalue file="classCorrade_1_1Utility_1_1Debug.html" anchor="aa346e932b5141006c49777ddacfaf600a533ddf198ed1163ad815ad0e4a2445f2">NoSpace</enumvalue>
      <enumvalue file="classCorrade_1_1Utility_1_1Debug.html" anchor="aa346e932b5141006c49777ddacfaf600aa1977c3f68d4d3bbfe14d0e51a575482">Packed</enumvalue>
      <enumvalue file="classCorrade_1_1Utility_1_1Debug.html" anchor="aa346e932b5141006c49777ddacfaf600acb5feb1b7314637725a2e73bdc9f7295">Color</enumvalue>
    </member>
    <member kind="typedef">
      <type>Containers::EnumSet&lt; Flag &gt;</type>
      <name>Flags</name>
      <anchorfile>classCorrade_1_1Utility_1_1Debug.html</anchorfile>
      <anchor>aedc4d6996ab47a86c346c1667794729b</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Debug</name>
      <anchorfile>classCorrade_1_1Utility_1_1Debug.html</anchorfile>
      <anchor>a069a145924bd86bfc3f4c6af07508452</anchor>
      <arglist>(Flags flags={})</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Debug</name>
      <anchorfile>classCorrade_1_1Utility_1_1Debug.html</anchorfile>
      <anchor>a094bd4c780a495f91dbe19776127f4f8</anchor>
      <arglist>(std::ostream *output, Flags flags={})</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Debug</name>
      <anchorfile>classCorrade_1_1Utility_1_1Debug.html</anchorfile>
      <anchor>a5eb811dee5401f2382acb8476f6d837a</anchor>
      <arglist>(const Debug &amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Debug</name>
      <anchorfile>classCorrade_1_1Utility_1_1Debug.html</anchorfile>
      <anchor>a11fa7c20ee652f1da6fe75d475ab0e48</anchor>
      <arglist>(Debug &amp;&amp;)=default</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~Debug</name>
      <anchorfile>classCorrade_1_1Utility_1_1Debug.html</anchorfile>
      <anchor>a4e30b5f23e9d3f56bb54942ef11288e8</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>Debug &amp;</type>
      <name>operator=</name>
      <anchorfile>classCorrade_1_1Utility_1_1Debug.html</anchorfile>
      <anchor>a621047711b250b30e87cf7143e429f0c</anchor>
      <arglist>(const Debug &amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type>Debug &amp;</type>
      <name>operator=</name>
      <anchorfile>classCorrade_1_1Utility_1_1Debug.html</anchorfile>
      <anchor>a02b17ddadb7259779431bafdb6dc84a2</anchor>
      <arglist>(Debug &amp;&amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type>Flags</type>
      <name>flags</name>
      <anchorfile>classCorrade_1_1Utility_1_1Debug.html</anchorfile>
      <anchor>a4817d2ebc83d92ed9a13b68937782c66</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setFlags</name>
      <anchorfile>classCorrade_1_1Utility_1_1Debug.html</anchorfile>
      <anchor>ab931824219ab68b12fec5a5c6aab1a4f</anchor>
      <arglist>(Flags flags)</arglist>
    </member>
    <member kind="function">
      <type>Flags</type>
      <name>immediateFlags</name>
      <anchorfile>classCorrade_1_1Utility_1_1Debug.html</anchorfile>
      <anchor>a6025547199d2f5d9fcd2cf3a0ea6386a</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setImmediateFlags</name>
      <anchorfile>classCorrade_1_1Utility_1_1Debug.html</anchorfile>
      <anchor>a9386e633a085278ca874c3aad19a3db1</anchor>
      <arglist>(Flags flags)</arglist>
    </member>
    <member kind="function">
      <type>Debug &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>classCorrade_1_1Utility_1_1Debug.html</anchorfile>
      <anchor>a713f25af5a460b8f3cf6e9b7203e8ebe</anchor>
      <arglist>(const char *value)</arglist>
    </member>
    <member kind="function">
      <type>Debug &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>classCorrade_1_1Utility_1_1Debug.html</anchorfile>
      <anchor>ab2b2895da70cbda6f19f62a0e92ad138</anchor>
      <arglist>(const void *value)</arglist>
    </member>
    <member kind="function">
      <type>Debug &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>classCorrade_1_1Utility_1_1Debug.html</anchorfile>
      <anchor>a4e34d1c961f28757b6cfe6ae214950e0</anchor>
      <arglist>(bool value)</arglist>
    </member>
    <member kind="function">
      <type>Debug &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>classCorrade_1_1Utility_1_1Debug.html</anchorfile>
      <anchor>a564fd5baa58847f23c7e5a0c392dbd41</anchor>
      <arglist>(char value)</arglist>
    </member>
    <member kind="function">
      <type>Debug &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>classCorrade_1_1Utility_1_1Debug.html</anchorfile>
      <anchor>a34be205387f75df9c973ac7541ab80da</anchor>
      <arglist>(unsigned char value)</arglist>
    </member>
    <member kind="function">
      <type>Debug &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>classCorrade_1_1Utility_1_1Debug.html</anchorfile>
      <anchor>a213a10e849be7d791c38f28fea654d63</anchor>
      <arglist>(int value)</arglist>
    </member>
    <member kind="function">
      <type>Debug &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>classCorrade_1_1Utility_1_1Debug.html</anchorfile>
      <anchor>af494a2b8261c74262966410ba0cd6677</anchor>
      <arglist>(long value)</arglist>
    </member>
    <member kind="function">
      <type>Debug &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>classCorrade_1_1Utility_1_1Debug.html</anchorfile>
      <anchor>a992bbaf961e11709a6f47c42872621e4</anchor>
      <arglist>(long long value)</arglist>
    </member>
    <member kind="function">
      <type>Debug &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>classCorrade_1_1Utility_1_1Debug.html</anchorfile>
      <anchor>ace984244c193e9c40a86f17df3e6be2b</anchor>
      <arglist>(unsigned value)</arglist>
    </member>
    <member kind="function">
      <type>Debug &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>classCorrade_1_1Utility_1_1Debug.html</anchorfile>
      <anchor>a5068548b7e32460f24b3e77cbc361ba0</anchor>
      <arglist>(unsigned long value)</arglist>
    </member>
    <member kind="function">
      <type>Debug &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>classCorrade_1_1Utility_1_1Debug.html</anchorfile>
      <anchor>ab5e5e420b2ffbe746fe5238740a89a47</anchor>
      <arglist>(unsigned long long value)</arglist>
    </member>
    <member kind="function">
      <type>Debug &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>classCorrade_1_1Utility_1_1Debug.html</anchorfile>
      <anchor>a64747bfa8b6b530d0d079d296d23eb33</anchor>
      <arglist>(float value)</arglist>
    </member>
    <member kind="function">
      <type>Debug &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>classCorrade_1_1Utility_1_1Debug.html</anchorfile>
      <anchor>a3bbb7769e9e3dd7a6cf59e22221c6d71</anchor>
      <arglist>(double value)</arglist>
    </member>
    <member kind="function">
      <type>Debug &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>classCorrade_1_1Utility_1_1Debug.html</anchorfile>
      <anchor>a27a4f78e38dcc3bdc8371da13c554038</anchor>
      <arglist>(long double value)</arglist>
    </member>
    <member kind="function">
      <type>Debug &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>classCorrade_1_1Utility_1_1Debug.html</anchorfile>
      <anchor>a580d97e0003cd26285ce8201687473ba</anchor>
      <arglist>(char32_t value)</arglist>
    </member>
    <member kind="function">
      <type>Debug &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>classCorrade_1_1Utility_1_1Debug.html</anchorfile>
      <anchor>ac29e10def861342c58687f4aa679b285</anchor>
      <arglist>(const char32_t *value)</arglist>
    </member>
    <member kind="function">
      <type>Debug &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>classCorrade_1_1Utility_1_1Debug.html</anchorfile>
      <anchor>adc97641f1b80496382e7b8bbf412aadb</anchor>
      <arglist>(std::nullptr_t)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static std::ostream *</type>
      <name>output</name>
      <anchorfile>classCorrade_1_1Utility_1_1Debug.html</anchorfile>
      <anchor>a8e86dc68a57d254e1a539b91672a496f</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static bool</type>
      <name>isTty</name>
      <anchorfile>classCorrade_1_1Utility_1_1Debug.html</anchorfile>
      <anchor>a35394a39a5df4322a36a598557c07db1</anchor>
      <arglist>(std::ostream *output)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static bool</type>
      <name>isTty</name>
      <anchorfile>classCorrade_1_1Utility_1_1Debug.html</anchorfile>
      <anchor>a32458743e3d3731036378cf8577c7934</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>Debug &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>classCorrade_1_1Utility_1_1Debug.html</anchorfile>
      <anchor>a1c2f055fab5c0e4e526e5bcd1c703735</anchor>
      <arglist>(Debug &amp;debug, Debug::Color value)</arglist>
    </member>
    <member kind="function">
      <type>Debug &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>classCorrade_1_1Utility_1_1Debug.html</anchorfile>
      <anchor>af0a2845a6cbbbe1962ed74491f261cc8</anchor>
      <arglist>(Debug &amp;debug, Debug::Flag value)</arglist>
    </member>
    <member kind="function">
      <type>Debug &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>classCorrade_1_1Utility_1_1Debug.html</anchorfile>
      <anchor>a9a67da2c7331baec84d75993e576f95d</anchor>
      <arglist>(Debug &amp;debug, Debug::Flags value)</arglist>
    </member>
    <member kind="function">
      <type>Debug &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>classCorrade_1_1Utility_1_1Debug.html</anchorfile>
      <anchor>ae2cee4795e84d13d29d1d9e1c805ecd2</anchor>
      <arglist>(Debug &amp;debug, const T &amp;value)</arglist>
    </member>
    <member kind="function">
      <type>Debug &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>classCorrade_1_1Utility_1_1Debug.html</anchorfile>
      <anchor>ac103e9217a8710bd03ee4783d3d6ed4b</anchor>
      <arglist>(Debug &amp;debug, const Iterable &amp;value)</arglist>
    </member>
    <member kind="function">
      <type>Debug &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>classCorrade_1_1Utility_1_1Debug.html</anchorfile>
      <anchor>a76d39b67ac31b0abf23b5c5d4bec9f15</anchor>
      <arglist>(Debug &amp;debug, const std::pair&lt; T, U &gt; &amp;value)</arglist>
    </member>
    <member kind="function">
      <type>Debug &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>classCorrade_1_1Utility_1_1Debug.html</anchorfile>
      <anchor>a516b71a8f99c9bd0edea7275889ebc7a</anchor>
      <arglist>(Debug &amp;debug, const std::string &amp;value)</arglist>
    </member>
    <member kind="function">
      <type>Debug &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>classCorrade_1_1Utility_1_1Debug.html</anchorfile>
      <anchor>af6d7c9ff8d1c4acaed2c0d5bfe8b4a63</anchor>
      <arglist>(Debug &amp;debug, const std::basic_string&lt; T &gt; &amp;value)</arglist>
    </member>
    <member kind="function">
      <type>Debug &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>classCorrade_1_1Utility_1_1Debug.html</anchorfile>
      <anchor>a9f9d27d6f040e6e490f2d73c5a0b8b56</anchor>
      <arglist>(Debug &amp;debug, const std::tuple&lt; Args... &gt; &amp;value)</arglist>
    </member>
    <member kind="enumeration">
      <type></type>
      <name>Color</name>
      <anchorfile>classCorrade_1_1Utility_1_1Debug.html</anchorfile>
      <anchor>a40f3e8edbd24f69725130f20202bbc2d</anchor>
      <arglist></arglist>
      <enumvalue file="classCorrade_1_1Utility_1_1Debug.html" anchor="a40f3e8edbd24f69725130f20202bbc2dae90dfb84e30edf611e326eeb04d680de">Black</enumvalue>
      <enumvalue file="classCorrade_1_1Utility_1_1Debug.html" anchor="a40f3e8edbd24f69725130f20202bbc2daee38e4d5dd68c4e440825018d549cb47">Red</enumvalue>
      <enumvalue file="classCorrade_1_1Utility_1_1Debug.html" anchor="a40f3e8edbd24f69725130f20202bbc2dad382816a3cbeed082c9e216e7392eed1">Green</enumvalue>
      <enumvalue file="classCorrade_1_1Utility_1_1Debug.html" anchor="a40f3e8edbd24f69725130f20202bbc2da51e6cd92b6c45f9affdc158ecca2b8b8">Yellow</enumvalue>
      <enumvalue file="classCorrade_1_1Utility_1_1Debug.html" anchor="a40f3e8edbd24f69725130f20202bbc2da9594eec95be70e7b1710f730fdda33d9">Blue</enumvalue>
      <enumvalue file="classCorrade_1_1Utility_1_1Debug.html" anchor="a40f3e8edbd24f69725130f20202bbc2dab91cc2c1416fcca942b61c7ac5b1a9ac">Magenta</enumvalue>
      <enumvalue file="classCorrade_1_1Utility_1_1Debug.html" anchor="a40f3e8edbd24f69725130f20202bbc2da023c239d2f2538f140a20e72c7b73f20">Cyan</enumvalue>
      <enumvalue file="classCorrade_1_1Utility_1_1Debug.html" anchor="a40f3e8edbd24f69725130f20202bbc2da25a81701fbfa4a1efdf660a950c1d006">White</enumvalue>
      <enumvalue file="classCorrade_1_1Utility_1_1Debug.html" anchor="a40f3e8edbd24f69725130f20202bbc2da7a1920d61156abc05a60135aefe8bc67">Default</enumvalue>
    </member>
    <member kind="typedef">
      <type>void(*</type>
      <name>Modifier</name>
      <anchorfile>classCorrade_1_1Utility_1_1Debug.html</anchorfile>
      <anchor>aede8a384033583c6b407e28134fd7c8a</anchor>
      <arglist>)(Debug &amp;)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>nospace</name>
      <anchorfile>classCorrade_1_1Utility_1_1Debug.html</anchorfile>
      <anchor>a025728a1744179cbae9b801920098047</anchor>
      <arglist>(Debug &amp;debug)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>newline</name>
      <anchorfile>classCorrade_1_1Utility_1_1Debug.html</anchorfile>
      <anchor>a8a96ee3513ec028eddbc520356293213</anchor>
      <arglist>(Debug &amp;debug)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static Modifier</type>
      <name>color</name>
      <anchorfile>classCorrade_1_1Utility_1_1Debug.html</anchorfile>
      <anchor>a4e59d103a5f719596fd14ea12677f928</anchor>
      <arglist>(Color color)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static Modifier</type>
      <name>boldColor</name>
      <anchorfile>classCorrade_1_1Utility_1_1Debug.html</anchorfile>
      <anchor>a522e0f0e38056d5592f4ae8b757ddfa6</anchor>
      <arglist>(Color color)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>resetColor</name>
      <anchorfile>classCorrade_1_1Utility_1_1Debug.html</anchorfile>
      <anchor>adf358e663180cb94e812f73dedc4f86c</anchor>
      <arglist>(Debug &amp;debug)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>packed</name>
      <anchorfile>classCorrade_1_1Utility_1_1Debug.html</anchorfile>
      <anchor>a7d729e8b4f5722c1026606ec0e5b6a21</anchor>
      <arglist>(Debug &amp;debug)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>color</name>
      <anchorfile>classCorrade_1_1Utility_1_1Debug.html</anchorfile>
      <anchor>a060a0a66096bfed18616de2540f5f5cd</anchor>
      <arglist>(Debug &amp;debug)</arglist>
    </member>
    <member kind="function">
      <type>Debug &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>classCorrade_1_1Utility_1_1Debug.html</anchorfile>
      <anchor>ac5f5e43d351399cc0dc1e471a753a300</anchor>
      <arglist>(Modifier f)</arglist>
    </member>
    <docanchor file="classCorrade_1_1Utility_1_1Debug" title="Printing STL types">Utility-Debug-stl</docanchor>
    <docanchor file="classCorrade_1_1Utility_1_1Debug" title="Scoped output redirection">Utility-Debug-scoped-output</docanchor>
    <docanchor file="classCorrade_1_1Utility_1_1Debug" title="Output modifiers">Utility-Debug-modifiers</docanchor>
    <docanchor file="classCorrade_1_1Utility_1_1Debug" title="Explicit whitespace control">Utility-Debug-modifiers-whitespace</docanchor>
    <docanchor file="classCorrade_1_1Utility_1_1Debug" title="Colored output">Utility-Debug-modifiers-colors</docanchor>
    <docanchor file="classCorrade_1_1Utility_1_1Debug" title="ANSI color support and UTF-8 output on Windows">Utility-Debug-windows</docanchor>
    <docanchor file="classCorrade_1_1Utility_1_1Debug" title="Thread safety">Utility-Debug-multithreading</docanchor>
  </compound>
  <compound kind="class">
    <name>Corrade::Utility::Endianness</name>
    <filename>classCorrade_1_1Utility_1_1Endianness.html</filename>
    <member kind="function" static="yes">
      <type>static T</type>
      <name>swap</name>
      <anchorfile>classCorrade_1_1Utility_1_1Endianness.html</anchorfile>
      <anchor>a37b31ac2f71e436d1d090b7d9a27973e</anchor>
      <arglist>(T value)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static constexpr bool</type>
      <name>isBigEndian</name>
      <anchorfile>classCorrade_1_1Utility_1_1Endianness.html</anchorfile>
      <anchor>a9d7a7670b74586ed27761f5d7c330ccc</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static T</type>
      <name>bigEndian</name>
      <anchorfile>classCorrade_1_1Utility_1_1Endianness.html</anchorfile>
      <anchor>ab9447075a4f52305ecc520228a3520ff</anchor>
      <arglist>(T value)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>bigEndianInPlace</name>
      <anchorfile>classCorrade_1_1Utility_1_1Endianness.html</anchorfile>
      <anchor>a0132a1001723d51ee3b3bc20da104285</anchor>
      <arglist>(T &amp;... values)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static T</type>
      <name>littleEndian</name>
      <anchorfile>classCorrade_1_1Utility_1_1Endianness.html</anchorfile>
      <anchor>a14ff651446505859b617f2daaf94e0f0</anchor>
      <arglist>(T number)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>littleEndianInPlace</name>
      <anchorfile>classCorrade_1_1Utility_1_1Endianness.html</anchorfile>
      <anchor>adb98231c5a674e212695646d1ddb386f</anchor>
      <arglist>(T &amp;... values)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>Corrade::Utility::Error</name>
    <filename>classCorrade_1_1Utility_1_1Error.html</filename>
    <base>Corrade::Utility::Debug</base>
    <member kind="function">
      <type></type>
      <name>Error</name>
      <anchorfile>classCorrade_1_1Utility_1_1Error.html</anchorfile>
      <anchor>a3fbe9b0f2557ce83055671246dcf7e7e</anchor>
      <arglist>(Flags flags={})</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Error</name>
      <anchorfile>classCorrade_1_1Utility_1_1Error.html</anchorfile>
      <anchor>a213951517715c247dac75d596c2d13f8</anchor>
      <arglist>(std::ostream *output, Flags flags={})</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Error</name>
      <anchorfile>classCorrade_1_1Utility_1_1Error.html</anchorfile>
      <anchor>aabe83634a34265d09ec611d1d9e21914</anchor>
      <arglist>(const Error &amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Error</name>
      <anchorfile>classCorrade_1_1Utility_1_1Error.html</anchorfile>
      <anchor>a792f4425e6734e77e99ae29be4fb9572</anchor>
      <arglist>(Error &amp;&amp;)=default</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~Error</name>
      <anchorfile>classCorrade_1_1Utility_1_1Error.html</anchorfile>
      <anchor>aea88b9362728fd3306d67b80b65994a7</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>Error &amp;</type>
      <name>operator=</name>
      <anchorfile>classCorrade_1_1Utility_1_1Error.html</anchorfile>
      <anchor>a1283baaa3477cbc49d48bfad02d57e5a</anchor>
      <arglist>(const Error &amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type>Error &amp;</type>
      <name>operator=</name>
      <anchorfile>classCorrade_1_1Utility_1_1Error.html</anchorfile>
      <anchor>a23c200ee5c7044cd25ce4d32a23d5f7a</anchor>
      <arglist>(Error &amp;&amp;)=delete</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static std::ostream *</type>
      <name>output</name>
      <anchorfile>classCorrade_1_1Utility_1_1Error.html</anchorfile>
      <anchor>aa9d822cae6bdb9632ff49fb8b8ca8b02</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static bool</type>
      <name>isTty</name>
      <anchorfile>classCorrade_1_1Utility_1_1Error.html</anchorfile>
      <anchor>a8b9c5f47adfa98178a8830959aa14da1</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>Corrade::Utility::Fatal</name>
    <filename>classCorrade_1_1Utility_1_1Fatal.html</filename>
    <base>Corrade::Utility::Error</base>
    <member kind="function">
      <type></type>
      <name>Fatal</name>
      <anchorfile>classCorrade_1_1Utility_1_1Fatal.html</anchorfile>
      <anchor>ace9441b319771c44b9407736733d9489</anchor>
      <arglist>(int exitCode=1, Flags flags={})</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Fatal</name>
      <anchorfile>classCorrade_1_1Utility_1_1Fatal.html</anchorfile>
      <anchor>a2b8532632da13ea29fe0aac2f8cc30d5</anchor>
      <arglist>(Flags flags)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Fatal</name>
      <anchorfile>classCorrade_1_1Utility_1_1Fatal.html</anchorfile>
      <anchor>acd4ec7626fa35d43d0d7453d34fe7874</anchor>
      <arglist>(std::ostream *output, int exitCode=1, Flags flags={})</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Fatal</name>
      <anchorfile>classCorrade_1_1Utility_1_1Fatal.html</anchorfile>
      <anchor>a5960d6dab2e70b5c8244a2d263ee5ab0</anchor>
      <arglist>(std::ostream *output, Flags flags={})</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~Fatal</name>
      <anchorfile>classCorrade_1_1Utility_1_1Fatal.html</anchorfile>
      <anchor>a7a65b11e97ce23312a7c62e9d52cd847</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>Corrade::Utility::FileWatcher</name>
    <filename>classCorrade_1_1Utility_1_1FileWatcher.html</filename>
    <member kind="function">
      <type></type>
      <name>FileWatcher</name>
      <anchorfile>classCorrade_1_1Utility_1_1FileWatcher.html</anchorfile>
      <anchor>a44f2c5a015035f2dd1b32633cfe6b8a0</anchor>
      <arglist>(const std::string &amp;filename)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>FileWatcher</name>
      <anchorfile>classCorrade_1_1Utility_1_1FileWatcher.html</anchorfile>
      <anchor>a340047f7af7e89e54c3ed55e5aaae479</anchor>
      <arglist>(const FileWatcher &amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>FileWatcher</name>
      <anchorfile>classCorrade_1_1Utility_1_1FileWatcher.html</anchorfile>
      <anchor>a9b793fb93618bf4909373410d1484632</anchor>
      <arglist>(FileWatcher &amp;&amp;) noexcept</arglist>
    </member>
    <member kind="function">
      <type>FileWatcher &amp;</type>
      <name>operator=</name>
      <anchorfile>classCorrade_1_1Utility_1_1FileWatcher.html</anchorfile>
      <anchor>a2f7a150a0f6e3db90eb46bbaa9a9fa2f</anchor>
      <arglist>(const FileWatcher &amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type>FileWatcher &amp;</type>
      <name>operator=</name>
      <anchorfile>classCorrade_1_1Utility_1_1FileWatcher.html</anchorfile>
      <anchor>aa181412c650e85e8a3b2b59af1a4834a</anchor>
      <arglist>(FileWatcher &amp;&amp;) noexcept</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>isValid</name>
      <anchorfile>classCorrade_1_1Utility_1_1FileWatcher.html</anchorfile>
      <anchor>a3ed4c9372013ecd8666786ed97efe70d</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>hasChanged</name>
      <anchorfile>classCorrade_1_1Utility_1_1FileWatcher.html</anchorfile>
      <anchor>a117f57738edcfc88011cc077a70d60e0</anchor>
      <arglist>()</arglist>
    </member>
    <docanchor file="classCorrade_1_1Utility_1_1FileWatcher" title="Behavior">Utility-FileWatcher-behavior</docanchor>
  </compound>
  <compound kind="class">
    <name>Corrade::Utility::HashDigest</name>
    <filename>classCorrade_1_1Utility_1_1HashDigest.html</filename>
    <templarg>size</templarg>
    <member kind="function">
      <type>constexpr</type>
      <name>HashDigest</name>
      <anchorfile>classCorrade_1_1Utility_1_1HashDigest.html</anchorfile>
      <anchor>abb77f485b60ebc66137e58945ed1d27e</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>HashDigest</name>
      <anchorfile>classCorrade_1_1Utility_1_1HashDigest.html</anchorfile>
      <anchor>a3479f9469f58d5042e9b7464c786b5eb</anchor>
      <arglist>(T firstValue, U... nextValues)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator==</name>
      <anchorfile>classCorrade_1_1Utility_1_1HashDigest.html</anchorfile>
      <anchor>ac5287774e7d16e7666984742b12fbe2c</anchor>
      <arglist>(const HashDigest&lt; size &gt; &amp;other) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator!=</name>
      <anchorfile>classCorrade_1_1Utility_1_1HashDigest.html</anchorfile>
      <anchor>a0ee8a3c5e1c90eebe4abdc1d1bd143f2</anchor>
      <arglist>(const HashDigest&lt; size &gt; &amp;other) const</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>hexString</name>
      <anchorfile>classCorrade_1_1Utility_1_1HashDigest.html</anchorfile>
      <anchor>a5aa68a903e80ba0af0f67d92c7103121</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr const char *</type>
      <name>byteArray</name>
      <anchorfile>classCorrade_1_1Utility_1_1HashDigest.html</anchorfile>
      <anchor>a2637d304d04a574d22e7326ecdc688c4</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static HashDigest&lt; size &gt;</type>
      <name>fromHexString</name>
      <anchorfile>classCorrade_1_1Utility_1_1HashDigest.html</anchorfile>
      <anchor>a195da27f46772170af3ef3b647c52361</anchor>
      <arglist>(std::string digest)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static const HashDigest&lt; size &gt; &amp;</type>
      <name>fromByteArray</name>
      <anchorfile>classCorrade_1_1Utility_1_1HashDigest.html</anchorfile>
      <anchor>add330139edc8634f4647832bff793b0c</anchor>
      <arglist>(const char *digest)</arglist>
    </member>
    <member kind="function">
      <type>Debug &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>classCorrade_1_1Utility_1_1HashDigest.html</anchorfile>
      <anchor>a39667d2dab7d417582ae04c76fd8af78</anchor>
      <arglist>(Debug &amp;debug, const HashDigest&lt; size &gt; &amp;value)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>Corrade::Utility::MurmurHash2</name>
    <filename>classCorrade_1_1Utility_1_1MurmurHash2.html</filename>
    <base>AbstractHash&lt; sizeof(std::size_t)&gt;</base>
    <member kind="function">
      <type>constexpr</type>
      <name>MurmurHash2</name>
      <anchorfile>classCorrade_1_1Utility_1_1MurmurHash2.html</anchorfile>
      <anchor>a7b512789e9557ebc5d04429af1b29c85</anchor>
      <arglist>(std::size_t seed=0)</arglist>
    </member>
    <member kind="function">
      <type>Digest</type>
      <name>operator()</name>
      <anchorfile>classCorrade_1_1Utility_1_1MurmurHash2.html</anchorfile>
      <anchor>a44f686e3054c52d362958e540279331b</anchor>
      <arglist>(const std::string &amp;data) const</arglist>
    </member>
    <member kind="function">
      <type>Digest</type>
      <name>operator()</name>
      <anchorfile>classCorrade_1_1Utility_1_1MurmurHash2.html</anchorfile>
      <anchor>a32b959a6e787dacceb02df824ed8925c</anchor>
      <arglist>(const char(&amp;data)[size]) const</arglist>
    </member>
    <member kind="function">
      <type>Digest</type>
      <name>operator()</name>
      <anchorfile>classCorrade_1_1Utility_1_1MurmurHash2.html</anchorfile>
      <anchor>a2519c23d08fc9f961a16b91ad84df152</anchor>
      <arglist>(const char *data, std::size_t size) const</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static Digest</type>
      <name>digest</name>
      <anchorfile>classCorrade_1_1Utility_1_1MurmurHash2.html</anchorfile>
      <anchor>abca6f6111dc7e38921fc15ba22ff885b</anchor>
      <arglist>(const std::string &amp;data)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>Corrade::Utility::Resource</name>
    <filename>classCorrade_1_1Utility_1_1Resource.html</filename>
    <member kind="function">
      <type></type>
      <name>Resource</name>
      <anchorfile>classCorrade_1_1Utility_1_1Resource.html</anchorfile>
      <anchor>afbf57511408670c4e52d987fae39d086</anchor>
      <arglist>(const std::string &amp;group)</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; std::string &gt;</type>
      <name>list</name>
      <anchorfile>classCorrade_1_1Utility_1_1Resource.html</anchorfile>
      <anchor>a1af41e4347a6350d2c98ed8950b21c74</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>Containers::ArrayView&lt; const char &gt;</type>
      <name>getRaw</name>
      <anchorfile>classCorrade_1_1Utility_1_1Resource.html</anchorfile>
      <anchor>a8caa576ca24c210054c37a28d01bdef7</anchor>
      <arglist>(const std::string &amp;filename) const</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>get</name>
      <anchorfile>classCorrade_1_1Utility_1_1Resource.html</anchorfile>
      <anchor>a5896c83252c7311ffccc4c6b49991fcf</anchor>
      <arglist>(const std::string &amp;filename) const</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static std::string</type>
      <name>compile</name>
      <anchorfile>classCorrade_1_1Utility_1_1Resource.html</anchorfile>
      <anchor>ae27c0dcd044bf1ed9a4778a12cf73735</anchor>
      <arglist>(const std::string &amp;name, const std::string &amp;group, const std::vector&lt; std::pair&lt; std::string, std::string &gt;&gt; &amp;files)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static std::string</type>
      <name>compileFrom</name>
      <anchorfile>classCorrade_1_1Utility_1_1Resource.html</anchorfile>
      <anchor>a19b48421cde009ddd1fa04a2e1b4ff0e</anchor>
      <arglist>(const std::string &amp;name, const std::string &amp;configurationFile)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>overrideGroup</name>
      <anchorfile>classCorrade_1_1Utility_1_1Resource.html</anchorfile>
      <anchor>a24f3eacc460abf21d6a3a439bba988cb</anchor>
      <arglist>(const std::string &amp;group, const std::string &amp;configurationFile)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static bool</type>
      <name>hasGroup</name>
      <anchorfile>classCorrade_1_1Utility_1_1Resource.html</anchorfile>
      <anchor>a4e1b9ed0b2f0f744035ec5280217befe</anchor>
      <arglist>(const std::string &amp;group)</arglist>
    </member>
    <docanchor file="classCorrade_1_1Utility_1_1Resource" title="Resource configuration file">Utility-Resource-conf</docanchor>
  </compound>
  <compound kind="class">
    <name>Corrade::Utility::Sha1</name>
    <filename>classCorrade_1_1Utility_1_1Sha1.html</filename>
    <base>AbstractHash&lt; 20 &gt;</base>
    <member kind="function">
      <type>Sha1 &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>classCorrade_1_1Utility_1_1Sha1.html</anchorfile>
      <anchor>aacf2b0b3f154574aa69951e8e008584c</anchor>
      <arglist>(const std::string &amp;data)</arglist>
    </member>
    <member kind="function">
      <type>Digest</type>
      <name>digest</name>
      <anchorfile>classCorrade_1_1Utility_1_1Sha1.html</anchorfile>
      <anchor>a9f933c6680b1dcf1bf185429c8b571ec</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static Digest</type>
      <name>digest</name>
      <anchorfile>classCorrade_1_1Utility_1_1Sha1.html</anchorfile>
      <anchor>a58ae2de7f39775cb7fb3594b80e98be5</anchor>
      <arglist>(const std::string &amp;data)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>Corrade::Utility::Tweakable</name>
    <filename>classCorrade_1_1Utility_1_1Tweakable.html</filename>
    <member kind="function">
      <type></type>
      <name>Tweakable</name>
      <anchorfile>classCorrade_1_1Utility_1_1Tweakable.html</anchorfile>
      <anchor>a584fb174e821bb3abc138a6303f65c54</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Tweakable</name>
      <anchorfile>classCorrade_1_1Utility_1_1Tweakable.html</anchorfile>
      <anchor>a634be6f49473281bcbf832dbe3fa3f02</anchor>
      <arglist>(const Tweakable &amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Tweakable</name>
      <anchorfile>classCorrade_1_1Utility_1_1Tweakable.html</anchorfile>
      <anchor>a762069362097d5d23ff4fdbe5d5044ed</anchor>
      <arglist>(Tweakable &amp;&amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type>Tweakable &amp;</type>
      <name>operator=</name>
      <anchorfile>classCorrade_1_1Utility_1_1Tweakable.html</anchorfile>
      <anchor>aaf42ad7adaa13aad54ca521edf5b9f0b</anchor>
      <arglist>(const Tweakable &amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type>Tweakable &amp;</type>
      <name>operator=</name>
      <anchorfile>classCorrade_1_1Utility_1_1Tweakable.html</anchorfile>
      <anchor>a37116ed525e0ba6f63b2f55dcd1f5432</anchor>
      <arglist>(Tweakable &amp;&amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~Tweakable</name>
      <anchorfile>classCorrade_1_1Utility_1_1Tweakable.html</anchorfile>
      <anchor>add3ffd194ca3b491617c70f87554d21f</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>isEnabled</name>
      <anchorfile>classCorrade_1_1Utility_1_1Tweakable.html</anchorfile>
      <anchor>a3bc8cadf88a529a1c4068d9588977551</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>enable</name>
      <anchorfile>classCorrade_1_1Utility_1_1Tweakable.html</anchorfile>
      <anchor>afd49c21bb2004744da97f93dfd5fdb05</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>enable</name>
      <anchorfile>classCorrade_1_1Utility_1_1Tweakable.html</anchorfile>
      <anchor>ae5d1ab55b5ecb61fbb534e25ef780e7f</anchor>
      <arglist>(const std::string &amp;prefix, const std::string &amp;replace)</arglist>
    </member>
    <member kind="function">
      <type>TweakableState</type>
      <name>update</name>
      <anchorfile>classCorrade_1_1Utility_1_1Tweakable.html</anchorfile>
      <anchor>a7fb3f46459272592d8d8c1e22b9d5a45</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>scope</name>
      <anchorfile>classCorrade_1_1Utility_1_1Tweakable.html</anchorfile>
      <anchor>ab169345f695835b7dd3c8d97892d7528</anchor>
      <arglist>(void(*lambda)(T &amp;), T &amp;userData)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>scope</name>
      <anchorfile>classCorrade_1_1Utility_1_1Tweakable.html</anchorfile>
      <anchor>a5472dd6f31194bc3a422b2b73f38f9be</anchor>
      <arglist>(void(*lambda)(void *), void *userData=nullptr)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static Tweakable &amp;</type>
      <name>instance</name>
      <anchorfile>classCorrade_1_1Utility_1_1Tweakable.html</anchorfile>
      <anchor>a0813cb4dfcbaa8aa22926383e1b323c6</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CORRADE_TWEAKABLE</name>
      <anchorfile>classCorrade_1_1Utility_1_1Tweakable.html</anchorfile>
      <anchor>a1db8193e4bf2fd84d96b54832ec55b64</anchor>
      <arglist>(...)</arglist>
    </member>
    <docanchor file="classCorrade_1_1Utility_1_1Tweakable" title="Basic usage">Utility-Tweakable-usage</docanchor>
    <docanchor file="classCorrade_1_1Utility_1_1Tweakable" title="Using scopes">Utility-Tweakable-usage-scope</docanchor>
    <docanchor file="classCorrade_1_1Utility_1_1Tweakable" title="Disabling tweakable values">Utility-Tweakable-usage-disabling</docanchor>
    <docanchor file="classCorrade_1_1Utility_1_1Tweakable" title="Limitations">Utility-Tweakable-limitations</docanchor>
    <docanchor file="classCorrade_1_1Utility_1_1Tweakable" title="How it works">Utility-Tweakable-how-it-works</docanchor>
    <docanchor file="classCorrade_1_1Utility_1_1Tweakable" title="Extending for custom types">Utility-Tweakable-extending</docanchor>
    <docanchor file="classCorrade_1_1Utility_1_1Tweakable" title="References">Utility-Tweakable-references</docanchor>
  </compound>
  <compound kind="struct">
    <name>Corrade::Utility::TweakableParser</name>
    <filename>structCorrade_1_1Utility_1_1TweakableParser.html</filename>
    <templarg></templarg>
    <docanchor file="structCorrade_1_1Utility_1_1TweakableParser" title="Implementing support for custom literals">TweakableParser-subclassing</docanchor>
  </compound>
  <compound kind="struct">
    <name>Corrade::Utility::TweakableParser&lt; bool &gt;</name>
    <filename>structCorrade_1_1Utility_1_1TweakableParser_3_01bool_01_4.html</filename>
    <member kind="function" static="yes">
      <type>static std::pair&lt; TweakableState, bool &gt;</type>
      <name>parse</name>
      <anchorfile>structCorrade_1_1Utility_1_1TweakableParser_3_01bool_01_4.html</anchorfile>
      <anchor>a42b59217107a285e6ee2582861130b8f</anchor>
      <arglist>(Containers::ArrayView&lt; const char &gt; value)</arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>Corrade::Utility::TweakableParser&lt; char &gt;</name>
    <filename>structCorrade_1_1Utility_1_1TweakableParser_3_01char_01_4.html</filename>
    <member kind="function" static="yes">
      <type>static std::pair&lt; TweakableState, char &gt;</type>
      <name>parse</name>
      <anchorfile>structCorrade_1_1Utility_1_1TweakableParser_3_01char_01_4.html</anchorfile>
      <anchor>acba3a4086b833b8508bd8dd90e1edfed</anchor>
      <arglist>(Containers::ArrayView&lt; const char &gt; value)</arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>Corrade::Utility::TweakableParser&lt; double &gt;</name>
    <filename>structCorrade_1_1Utility_1_1TweakableParser_3_01double_01_4.html</filename>
    <member kind="function" static="yes">
      <type>static std::pair&lt; TweakableState, double &gt;</type>
      <name>parse</name>
      <anchorfile>structCorrade_1_1Utility_1_1TweakableParser_3_01double_01_4.html</anchorfile>
      <anchor>a23adabf166b8d243cddd516808aaeec1</anchor>
      <arglist>(Containers::ArrayView&lt; const char &gt; value)</arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>Corrade::Utility::TweakableParser&lt; float &gt;</name>
    <filename>structCorrade_1_1Utility_1_1TweakableParser_3_01float_01_4.html</filename>
    <member kind="function" static="yes">
      <type>static std::pair&lt; TweakableState, float &gt;</type>
      <name>parse</name>
      <anchorfile>structCorrade_1_1Utility_1_1TweakableParser_3_01float_01_4.html</anchorfile>
      <anchor>aebbd35f25beb7df947c24ffddc5be47b</anchor>
      <arglist>(Containers::ArrayView&lt; const char &gt; value)</arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>Corrade::Utility::TweakableParser&lt; int &gt;</name>
    <filename>structCorrade_1_1Utility_1_1TweakableParser_3_01int_01_4.html</filename>
    <member kind="function" static="yes">
      <type>static std::pair&lt; TweakableState, int &gt;</type>
      <name>parse</name>
      <anchorfile>structCorrade_1_1Utility_1_1TweakableParser_3_01int_01_4.html</anchorfile>
      <anchor>ae1e5a9beb0906ec2a9b761d22a2c7ee7</anchor>
      <arglist>(Containers::ArrayView&lt; const char &gt; value)</arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>Corrade::Utility::TweakableParser&lt; long &gt;</name>
    <filename>structCorrade_1_1Utility_1_1TweakableParser_3_01long_01_4.html</filename>
    <member kind="function" static="yes">
      <type>static std::pair&lt; TweakableState, long &gt;</type>
      <name>parse</name>
      <anchorfile>structCorrade_1_1Utility_1_1TweakableParser_3_01long_01_4.html</anchorfile>
      <anchor>a9ee71381ce18778c5f55adeadf48f127</anchor>
      <arglist>(Containers::ArrayView&lt; const char &gt; value)</arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>Corrade::Utility::TweakableParser&lt; long double &gt;</name>
    <filename>structCorrade_1_1Utility_1_1TweakableParser_3_01long_01double_01_4.html</filename>
    <member kind="function" static="yes">
      <type>static std::pair&lt; TweakableState, long double &gt;</type>
      <name>parse</name>
      <anchorfile>structCorrade_1_1Utility_1_1TweakableParser_3_01long_01double_01_4.html</anchorfile>
      <anchor>aa3f897779a1f532607af46d8852ebbc7</anchor>
      <arglist>(Containers::ArrayView&lt; const char &gt; value)</arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>Corrade::Utility::TweakableParser&lt; long long &gt;</name>
    <filename>structCorrade_1_1Utility_1_1TweakableParser_3_01long_01long_01_4.html</filename>
    <member kind="function" static="yes">
      <type>static std::pair&lt; TweakableState, long long &gt;</type>
      <name>parse</name>
      <anchorfile>structCorrade_1_1Utility_1_1TweakableParser_3_01long_01long_01_4.html</anchorfile>
      <anchor>a1d91aa67223c1a15de4daadeeb0f5dcc</anchor>
      <arglist>(Containers::ArrayView&lt; const char &gt; value)</arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>Corrade::Utility::TweakableParser&lt; unsigned int &gt;</name>
    <filename>structCorrade_1_1Utility_1_1TweakableParser_3_01unsigned_01int_01_4.html</filename>
    <member kind="function" static="yes">
      <type>static std::pair&lt; TweakableState, unsigned int &gt;</type>
      <name>parse</name>
      <anchorfile>structCorrade_1_1Utility_1_1TweakableParser_3_01unsigned_01int_01_4.html</anchorfile>
      <anchor>a8a79c25028cc654750b3c1d997031750</anchor>
      <arglist>(Containers::ArrayView&lt; const char &gt; value)</arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>Corrade::Utility::TweakableParser&lt; unsigned long &gt;</name>
    <filename>structCorrade_1_1Utility_1_1TweakableParser_3_01unsigned_01long_01_4.html</filename>
    <member kind="function" static="yes">
      <type>static std::pair&lt; TweakableState, unsigned long &gt;</type>
      <name>parse</name>
      <anchorfile>structCorrade_1_1Utility_1_1TweakableParser_3_01unsigned_01long_01_4.html</anchorfile>
      <anchor>a3cb8130b47a1ae92a0fe6fd5286becad</anchor>
      <arglist>(Containers::ArrayView&lt; const char &gt; value)</arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>Corrade::Utility::TweakableParser&lt; unsigned long long &gt;</name>
    <filename>structCorrade_1_1Utility_1_1TweakableParser_3_01unsigned_01long_01long_01_4.html</filename>
    <member kind="function" static="yes">
      <type>static std::pair&lt; TweakableState, unsigned long long &gt;</type>
      <name>parse</name>
      <anchorfile>structCorrade_1_1Utility_1_1TweakableParser_3_01unsigned_01long_01long_01_4.html</anchorfile>
      <anchor>a595682944063e7fa167a1f77c652afbc</anchor>
      <arglist>(Containers::ArrayView&lt; const char &gt; value)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>Corrade::Utility::Warning</name>
    <filename>classCorrade_1_1Utility_1_1Warning.html</filename>
    <base>Corrade::Utility::Debug</base>
    <member kind="function">
      <type></type>
      <name>Warning</name>
      <anchorfile>classCorrade_1_1Utility_1_1Warning.html</anchorfile>
      <anchor>a6afe64c565a1ebae35228b840e63c428</anchor>
      <arglist>(Flags flags={})</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Warning</name>
      <anchorfile>classCorrade_1_1Utility_1_1Warning.html</anchorfile>
      <anchor>a92f3ddd2c7e6caed6b6967d0f72c001b</anchor>
      <arglist>(std::ostream *output, Flags flags={})</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Warning</name>
      <anchorfile>classCorrade_1_1Utility_1_1Warning.html</anchorfile>
      <anchor>a742946b1583b9e147a1b502c647fc1ec</anchor>
      <arglist>(const Warning &amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Warning</name>
      <anchorfile>classCorrade_1_1Utility_1_1Warning.html</anchorfile>
      <anchor>accd8b00e770a072a122c4ffbc7b2373c</anchor>
      <arglist>(Warning &amp;&amp;)=default</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~Warning</name>
      <anchorfile>classCorrade_1_1Utility_1_1Warning.html</anchorfile>
      <anchor>a66833257bc5b0d126add892b19091c56</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>Warning &amp;</type>
      <name>operator=</name>
      <anchorfile>classCorrade_1_1Utility_1_1Warning.html</anchorfile>
      <anchor>a10bd7464c247b4df94adb4ec6ab26c6e</anchor>
      <arglist>(const Warning &amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type>Warning &amp;</type>
      <name>operator=</name>
      <anchorfile>classCorrade_1_1Utility_1_1Warning.html</anchorfile>
      <anchor>adf0d29c428d7f4a094debd445a5a1516</anchor>
      <arglist>(Warning &amp;&amp;)=delete</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static std::ostream *</type>
      <name>output</name>
      <anchorfile>classCorrade_1_1Utility_1_1Warning.html</anchorfile>
      <anchor>ad124e9124dfbfac55f9c6cf602d1910d</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static bool</type>
      <name>isTty</name>
      <anchorfile>classCorrade_1_1Utility_1_1Warning.html</anchorfile>
      <anchor>a5b0ee54b7afbcd80816575963bf01702</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>EnumSet&lt; InternalFlag &gt;</name>
    <filename>classCorrade_1_1Containers_1_1EnumSet.html</filename>
    <member kind="typedef">
      <type>InternalFlag</type>
      <name>Type</name>
      <anchorfile>classCorrade_1_1Containers_1_1EnumSet.html</anchorfile>
      <anchor>af79d534425b3439095c579944858bb4c</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::underlying_type&lt; InternalFlag &gt;::type</type>
      <name>UnderlyingType</name>
      <anchorfile>classCorrade_1_1Containers_1_1EnumSet.html</anchorfile>
      <anchor>ae98013f13d62ca0bac69d632905846c6</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>FullValue</name>
      <anchorfile>classCorrade_1_1Containers_1_1EnumSet.html</anchorfile>
      <anchor>a4bb5d41e20280cfae07a6c6f74e5b689a9e456595199d05472bf7927d329639b8</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>EnumSet</name>
      <anchorfile>classCorrade_1_1Containers_1_1EnumSet.html</anchorfile>
      <anchor>ac3820645815d705507170a3e507087b5</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>EnumSet</name>
      <anchorfile>classCorrade_1_1Containers_1_1EnumSet.html</anchorfile>
      <anchor>aff6e8c4b80a660f381b31a2ed5106695</anchor>
      <arglist>(InternalFlag value)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>EnumSet</name>
      <anchorfile>classCorrade_1_1Containers_1_1EnumSet.html</anchorfile>
      <anchor>adb0d3bd1925b399503dc745e51c245a6</anchor>
      <arglist>(NoInitT)</arglist>
    </member>
    <member kind="function">
      <type>constexpr bool</type>
      <name>operator==</name>
      <anchorfile>classCorrade_1_1Containers_1_1EnumSet.html</anchorfile>
      <anchor>a73828ac4e6d6125500733a71eccd0d94</anchor>
      <arglist>(EnumSet&lt; InternalFlag, typename std::underlying_type&lt; InternalFlag &gt;::type(~0) &gt; other) const</arglist>
    </member>
    <member kind="function">
      <type>constexpr bool</type>
      <name>operator!=</name>
      <anchorfile>classCorrade_1_1Containers_1_1EnumSet.html</anchorfile>
      <anchor>a8ae1892b96c6e781165475388e4cbc53</anchor>
      <arglist>(EnumSet&lt; InternalFlag, typename std::underlying_type&lt; InternalFlag &gt;::type(~0) &gt; other) const</arglist>
    </member>
    <member kind="function">
      <type>constexpr bool</type>
      <name>operator&gt;=</name>
      <anchorfile>classCorrade_1_1Containers_1_1EnumSet.html</anchorfile>
      <anchor>aeb270daa9a8afb3a80e57ef2cb2f8314</anchor>
      <arglist>(EnumSet&lt; InternalFlag, typename std::underlying_type&lt; InternalFlag &gt;::type(~0) &gt; other) const</arglist>
    </member>
    <member kind="function">
      <type>constexpr bool</type>
      <name>operator&lt;=</name>
      <anchorfile>classCorrade_1_1Containers_1_1EnumSet.html</anchorfile>
      <anchor>a3ac067add1634645e3d64803e5751459</anchor>
      <arglist>(EnumSet&lt; InternalFlag, typename std::underlying_type&lt; InternalFlag &gt;::type(~0) &gt; other) const</arglist>
    </member>
    <member kind="function">
      <type>constexpr EnumSet&lt; InternalFlag, typename std::underlying_type&lt; InternalFlag &gt;::type(~0) &gt;</type>
      <name>operator|</name>
      <anchorfile>classCorrade_1_1Containers_1_1EnumSet.html</anchorfile>
      <anchor>a89503e71c491ed8fe7bb9f86f48309c4</anchor>
      <arglist>(EnumSet&lt; InternalFlag, typename std::underlying_type&lt; InternalFlag &gt;::type(~0) &gt; other) const</arglist>
    </member>
    <member kind="function">
      <type>EnumSet&lt; InternalFlag, typename std::underlying_type&lt; InternalFlag &gt;::type(~0) &gt; &amp;</type>
      <name>operator|=</name>
      <anchorfile>classCorrade_1_1Containers_1_1EnumSet.html</anchorfile>
      <anchor>a0570b5800f24317a1b52b27d878b9bfa</anchor>
      <arglist>(EnumSet&lt; InternalFlag, typename std::underlying_type&lt; InternalFlag &gt;::type(~0) &gt; other)</arglist>
    </member>
    <member kind="function">
      <type>constexpr EnumSet&lt; InternalFlag, typename std::underlying_type&lt; InternalFlag &gt;::type(~0) &gt;</type>
      <name>operator &amp;</name>
      <anchorfile>classCorrade_1_1Containers_1_1EnumSet.html</anchorfile>
      <anchor>acb47a914ad8ee78f762d78de3ccd12de</anchor>
      <arglist>(EnumSet&lt; InternalFlag, typename std::underlying_type&lt; InternalFlag &gt;::type(~0) &gt; other) const</arglist>
    </member>
    <member kind="function">
      <type>EnumSet&lt; InternalFlag, typename std::underlying_type&lt; InternalFlag &gt;::type(~0) &gt; &amp;</type>
      <name>operator&amp;=</name>
      <anchorfile>classCorrade_1_1Containers_1_1EnumSet.html</anchorfile>
      <anchor>a882bec7925638c92d8491ce07108f398</anchor>
      <arglist>(EnumSet&lt; InternalFlag, typename std::underlying_type&lt; InternalFlag &gt;::type(~0) &gt; other)</arglist>
    </member>
    <member kind="function">
      <type>constexpr EnumSet&lt; InternalFlag, typename std::underlying_type&lt; InternalFlag &gt;::type(~0) &gt;</type>
      <name>operator^</name>
      <anchorfile>classCorrade_1_1Containers_1_1EnumSet.html</anchorfile>
      <anchor>a8b8c4ac2173541eb7572d66b64931512</anchor>
      <arglist>(EnumSet&lt; InternalFlag, typename std::underlying_type&lt; InternalFlag &gt;::type(~0) &gt; other) const</arglist>
    </member>
    <member kind="function">
      <type>EnumSet&lt; InternalFlag, typename std::underlying_type&lt; InternalFlag &gt;::type(~0) &gt; &amp;</type>
      <name>operator^=</name>
      <anchorfile>classCorrade_1_1Containers_1_1EnumSet.html</anchorfile>
      <anchor>afc5caf6f9260aedc01b5d532e4143e36</anchor>
      <arglist>(EnumSet&lt; InternalFlag, typename std::underlying_type&lt; InternalFlag &gt;::type(~0) &gt; other)</arglist>
    </member>
    <member kind="function">
      <type>constexpr EnumSet&lt; InternalFlag, typename std::underlying_type&lt; InternalFlag &gt;::type(~0) &gt;</type>
      <name>operator~</name>
      <anchorfile>classCorrade_1_1Containers_1_1EnumSet.html</anchorfile>
      <anchor>ae9d66612bd81553d5e9cab2dd00235ea</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>operator bool</name>
      <anchorfile>classCorrade_1_1Containers_1_1EnumSet.html</anchorfile>
      <anchor>aaa72f9d5d7ab6e51d66d3698206e7cc9</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>operator UnderlyingType</name>
      <anchorfile>classCorrade_1_1Containers_1_1EnumSet.html</anchorfile>
      <anchor>a49d01415abacaea3fd08be0a18accc54</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>Utility::Debug &amp;</type>
      <name>enumSetDebugOutput</name>
      <anchorfile>classCorrade_1_1Containers_1_1EnumSet.html</anchorfile>
      <anchor>a12c4ea794fa62c56969b1311e2b5c21b</anchor>
      <arglist>(Utility::Debug &amp;debug, EnumSet&lt; InternalFlag, typename std::underlying_type&lt; InternalFlag &gt;::type(~0) &gt; value, const char *empty, std::initializer_list&lt; InternalFlag &gt; enums)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>Optional&lt; Corrade::PluginManager::PluginMetadata &gt;</name>
    <filename>classCorrade_1_1Containers_1_1Optional.html</filename>
    <member kind="function">
      <type></type>
      <name>Optional</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>a8e6f9f4c19abb7a88932b06d150d80b7</anchor>
      <arglist>(NullOptT=NullOpt) noexcept</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Optional</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>ad6663b92a690b02da84cc8923be9504b</anchor>
      <arglist>(const Corrade::PluginManager::PluginMetadata &amp;value) noexcept(std::is_nothrow_copy_assignable&lt; Corrade::PluginManager::PluginMetadata &gt;::value)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Optional</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>ad6113b40aabedc4f6bfc3344abd4a17c</anchor>
      <arglist>(Corrade::PluginManager::PluginMetadata &amp;&amp;value) noexcept(std::is_nothrow_move_assignable&lt; Corrade::PluginManager::PluginMetadata &gt;::value)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Optional</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>a0032b85cf2cacff3e4cb6e59387d0fe0</anchor>
      <arglist>(InPlaceInitT, Args &amp;&amp;... args) noexcept(std::is_nothrow_constructible&lt; Corrade::PluginManager::PluginMetadata, Args &amp;&amp;... &gt;::value)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Optional</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>a0b40ee5091561043f7b3db2105df0521</anchor>
      <arglist>(const U &amp;other) noexcept(std::is_nothrow_copy_constructible&lt; Corrade::PluginManager::PluginMetadata &gt;::value)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Optional</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>a3b40412d5eabf3aef81e871a1d8800dd</anchor>
      <arglist>(U &amp;&amp;other) noexcept(std::is_nothrow_move_constructible&lt; Corrade::PluginManager::PluginMetadata &gt;::value)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Optional</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>aa9d04a3b7e025d6428552ee7a7b572af</anchor>
      <arglist>(const Optional&lt; Corrade::PluginManager::PluginMetadata &gt; &amp;other) noexcept(std::is_nothrow_copy_constructible&lt; Corrade::PluginManager::PluginMetadata &gt;::value)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Optional</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>a89c8a0260c48815b8938484ae455c592</anchor>
      <arglist>(Optional&lt; Corrade::PluginManager::PluginMetadata &gt; &amp;&amp;other) noexcept(std::is_nothrow_move_constructible&lt; Corrade::PluginManager::PluginMetadata &gt;::value)</arglist>
    </member>
    <member kind="function">
      <type>Optional&lt; Corrade::PluginManager::PluginMetadata &gt; &amp;</type>
      <name>operator=</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>a2dcb464863ceb583033a012f53c3a7ad</anchor>
      <arglist>(const Optional&lt; Corrade::PluginManager::PluginMetadata &gt; &amp;other) noexcept(std::is_nothrow_copy_assignable&lt; Corrade::PluginManager::PluginMetadata &gt;::value)</arglist>
    </member>
    <member kind="function">
      <type>Optional&lt; Corrade::PluginManager::PluginMetadata &gt; &amp;</type>
      <name>operator=</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>a6946a7975796cd2c4e6472e33ee679c6</anchor>
      <arglist>(Optional&lt; Corrade::PluginManager::PluginMetadata &gt; &amp;&amp;other) noexcept(std::is_nothrow_move_assignable&lt; Corrade::PluginManager::PluginMetadata &gt;::value)</arglist>
    </member>
    <member kind="function">
      <type>Optional&lt; Corrade::PluginManager::PluginMetadata &gt; &amp;</type>
      <name>operator=</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>a36e2f775d6c9e4a209b24d1dda3f19b6</anchor>
      <arglist>(NullOptT) noexcept</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>operator U</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>a1bdc77d731bcc50845234e56450117e9</anchor>
      <arglist>() const &amp;</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>operator U</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>af1d5a8614a3d524db3263cc18494c9c5</anchor>
      <arglist>() &amp;&amp;</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~Optional</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>aca99a8049370de23ee82834babab94ee</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>operator bool</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>a389cd0c17eac32b9194f922fccc91934</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator==</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>a5845d1d681735a657392eb2b9bdd7e4b</anchor>
      <arglist>(const Optional&lt; Corrade::PluginManager::PluginMetadata &gt; &amp;other) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator==</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>a54979ab51051dfd5827c7cd59ef9595b</anchor>
      <arglist>(NullOptT) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator==</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>a73e7c08fd5a5dfe4c88ada7f145bc3fe</anchor>
      <arglist>(const Corrade::PluginManager::PluginMetadata &amp;other) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator!=</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>afd0801e25b3bf275106b816bb43e27f4</anchor>
      <arglist>(const Optional&lt; Corrade::PluginManager::PluginMetadata &gt; &amp;other) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator!=</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>aace07343ad31c9b87fa8e18f77c1c629</anchor>
      <arglist>(NullOptT) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator!=</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>ad95905fb8f63ad6a56d75aa82c4ad53f</anchor>
      <arglist>(const Corrade::PluginManager::PluginMetadata &amp;other) const</arglist>
    </member>
    <member kind="function">
      <type>Corrade::PluginManager::PluginMetadata *</type>
      <name>operator-&gt;</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>a000cfe22c91613c0acfb0816d06a3f5e</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const Corrade::PluginManager::PluginMetadata *</type>
      <name>operator-&gt;</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>a0816ce03c981b5cf64644973619bc861</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>Corrade::PluginManager::PluginMetadata &amp;</type>
      <name>operator *</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>aa4ff7c7b3c3f3dbe8e8e686078f344e0</anchor>
      <arglist>() &amp;</arglist>
    </member>
    <member kind="function">
      <type>Corrade::PluginManager::PluginMetadata &amp;&amp;</type>
      <name>operator *</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>a9168523f342bced22853cd8b583c8d7f</anchor>
      <arglist>() &amp;&amp;</arglist>
    </member>
    <member kind="function">
      <type>const Corrade::PluginManager::PluginMetadata &amp;</type>
      <name>operator *</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>a19e368fa818aaf235edd59ff53942a6a</anchor>
      <arglist>() const &amp;</arglist>
    </member>
    <member kind="function">
      <type>const Corrade::PluginManager::PluginMetadata &amp;&amp;</type>
      <name>operator *</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>a559026190509308f4fbf6667bb4a404c</anchor>
      <arglist>() const &amp;&amp;</arglist>
    </member>
    <member kind="function">
      <type>Corrade::PluginManager::PluginMetadata &amp;</type>
      <name>emplace</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>a28d3d69d7026de71a44797101d29f2ab</anchor>
      <arglist>(Args &amp;&amp;... args)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator==</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>a8404f64fbb8bf4965462b97179e4ebaa</anchor>
      <arglist>(NullOptT, const Optional&lt; Corrade::PluginManager::PluginMetadata &gt; &amp;b)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator==</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>aba65d386a0b9f9cfd54919c48bbd87e8</anchor>
      <arglist>(const Corrade::PluginManager::PluginMetadata &amp;a, const Optional&lt; Corrade::PluginManager::PluginMetadata &gt; &amp;b)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator!=</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>a1c553d2af62a154653128d1d9ce46a7f</anchor>
      <arglist>(NullOptT, const Optional&lt; Corrade::PluginManager::PluginMetadata &gt; &amp;b)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator!=</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>a19c44b59df82dbebce8c720ec88717f5</anchor>
      <arglist>(const Corrade::PluginManager::PluginMetadata &amp;a, const Optional&lt; Corrade::PluginManager::PluginMetadata &gt; &amp;b)</arglist>
    </member>
    <member kind="function">
      <type>Optional&lt; typename std::decay&lt; Corrade::PluginManager::PluginMetadata &gt;::type &gt;</type>
      <name>optional</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>a71e22f3640eb28dd7b2f7b8339931224</anchor>
      <arglist>(Corrade::PluginManager::PluginMetadata &amp;&amp;value)</arglist>
    </member>
    <member kind="function">
      <type>Optional&lt; Corrade::PluginManager::PluginMetadata &gt;</type>
      <name>optional</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>a828d28109eb44c02f39b554a392606e9</anchor>
      <arglist>(Args &amp;&amp;... args)</arglist>
    </member>
    <member kind="function">
      <type>auto</type>
      <name>optional</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>ab0ccf001156df9d35c557ce79a66b927</anchor>
      <arglist>(Corrade::PluginManager::PluginMetadata &amp;&amp;other) -&gt; decltype(Implementation::DeducedOptionalConverter&lt; typename std::decay&lt; Corrade::PluginManager::PluginMetadata &gt;::type &gt;::from(std::forward&lt; Corrade::PluginManager::PluginMetadata &gt;(other)))</arglist>
    </member>
    <member kind="function">
      <type>Utility::Debug &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>classCorrade_1_1Containers_1_1Optional.html</anchorfile>
      <anchor>abb61d1d1f8de79a08d22cef7ef99cb36</anchor>
      <arglist>(Utility::Debug &amp;debug, const Optional&lt; Corrade::PluginManager::PluginMetadata &gt; &amp;value)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>Pointer&lt; Data &gt;</name>
    <filename>classCorrade_1_1Containers_1_1Pointer.html</filename>
    <member kind="function">
      <type></type>
      <name>Pointer</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>adfca41b457c65ad6f7a20e4da25a551d</anchor>
      <arglist>(std::nullptr_t=nullptr) noexcept</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Pointer</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>a6961c1d838ce004acce16785ec434c91</anchor>
      <arglist>(Data *pointer) noexcept</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Pointer</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>afa90d4aa96c05da96180f20e3afda237</anchor>
      <arglist>(InPlaceInitT, Args &amp;&amp;... args)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Pointer</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>a30e35c7e582758f8814c2f6b6801dce8</anchor>
      <arglist>(Pointer&lt; U &gt; &amp;&amp;other) noexcept</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Pointer</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>a4e598da5a54d78e0f6e3802a3678ccde</anchor>
      <arglist>(U &amp;&amp;other) noexcept</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Pointer</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>ab3fc3f6972074b386bf623601dfcbebd</anchor>
      <arglist>(const Pointer&lt; Data &gt; &amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Pointer</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>a7cf60434462a0199b5bda1ecf4f80c01</anchor>
      <arglist>(Pointer&lt; Data &gt; &amp;&amp;other) noexcept</arglist>
    </member>
    <member kind="function">
      <type>Pointer&lt; Data &gt; &amp;</type>
      <name>operator=</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>af50cfd42d6d0cbdd0909f66b7d99c8db</anchor>
      <arglist>(const Pointer&lt; Data &gt; &amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type>Pointer&lt; Data &gt; &amp;</type>
      <name>operator=</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>ac18ee6f61f98e309e478725c61079a75</anchor>
      <arglist>(Pointer&lt; Data &gt; &amp;&amp;other) noexcept</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>operator U</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>a26d98d69b279f2618edf0785882e01e5</anchor>
      <arglist>() &amp;&amp;</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator==</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>a5b3089b6c7e3e57b3ebc712b3a1b65a5</anchor>
      <arglist>(std::nullptr_t) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator!=</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>aded8d11d6271891c1fa62572e59bc6dd</anchor>
      <arglist>(std::nullptr_t) const</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~Pointer</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>a3e4509891d92dde6088f5a73b0881a64</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>operator bool</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>a6f4df0db94fe387105c56d0691a25ac4</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>Data *</type>
      <name>get</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>a341521cc065c95861ed82d85e26a33d1</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const Data *</type>
      <name>get</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>a98b642e46b44b5a4d10a36e216470783</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>Data *</type>
      <name>operator-&gt;</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>a52a73243270252c6f91851b377958e62</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const Data *</type>
      <name>operator-&gt;</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>ae225b88b0d07991e9fff931bfab2f5d9</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>Data &amp;</type>
      <name>operator *</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>a047a241d186b89fe086929927e38b50a</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const Data &amp;</type>
      <name>operator *</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>ab04fa2ee84e6d3c6896b715c95d36a0d</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>reset</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>ac4150a4c7336f33d3e0bdbec832154ab</anchor>
      <arglist>(Data *pointer=nullptr)</arglist>
    </member>
    <member kind="function">
      <type>Data &amp;</type>
      <name>emplace</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>a76f18ffe7324c11f56cf073ca2737717</anchor>
      <arglist>(Args &amp;&amp;... args)</arglist>
    </member>
    <member kind="function">
      <type>Data *</type>
      <name>release</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>aaa8b2ed64ca7afb46c614e7aea7893a9</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator==</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>af9b9640eac6d17650d0cc5111c4a19b6</anchor>
      <arglist>(std::nullptr_t, const Pointer&lt; Data &gt; &amp;b)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator!=</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>a141814794f6072316c8fa72afc8c7f76</anchor>
      <arglist>(std::nullptr_t, const Pointer&lt; Data &gt; &amp;b)</arglist>
    </member>
    <member kind="function">
      <type>Pointer&lt; Data &gt;</type>
      <name>pointer</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>a76b1a48222910763fdc5046b6625090e</anchor>
      <arglist>(Data *pointer)</arglist>
    </member>
    <member kind="function">
      <type>auto</type>
      <name>pointer</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>a26b8a841d97121802988aff08bcbb0c6</anchor>
      <arglist>(Data &amp;&amp;other) -&gt; decltype(Implementation::DeducedPointerConverter&lt; Data &gt;::from(std::move(other)))</arglist>
    </member>
    <member kind="function">
      <type>Pointer&lt; Data &gt;</type>
      <name>pointer</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>ab287dc10fa2591e662d9a9ea611a3dad</anchor>
      <arglist>(Args &amp;&amp;... args)</arglist>
    </member>
    <member kind="function">
      <type>Pointer&lt; U &gt;</type>
      <name>pointerCast</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>ae1b70d106c3a13b76188c0f7f956cb0c</anchor>
      <arglist>(Pointer&lt; Data &gt; &amp;&amp;pointer)</arglist>
    </member>
    <member kind="function">
      <type>Utility::Debug &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>a9c784a034b942686ae70ce8f5b237e26</anchor>
      <arglist>(Utility::Debug &amp;debug, const Pointer&lt; Data &gt; &amp;value)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>Pointer&lt; State &gt;</name>
    <filename>classCorrade_1_1Containers_1_1Pointer.html</filename>
    <member kind="function">
      <type></type>
      <name>Pointer</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>adfca41b457c65ad6f7a20e4da25a551d</anchor>
      <arglist>(std::nullptr_t=nullptr) noexcept</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Pointer</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>a6961c1d838ce004acce16785ec434c91</anchor>
      <arglist>(State *pointer) noexcept</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Pointer</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>afa90d4aa96c05da96180f20e3afda237</anchor>
      <arglist>(InPlaceInitT, Args &amp;&amp;... args)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Pointer</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>a30e35c7e582758f8814c2f6b6801dce8</anchor>
      <arglist>(Pointer&lt; U &gt; &amp;&amp;other) noexcept</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Pointer</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>a4e598da5a54d78e0f6e3802a3678ccde</anchor>
      <arglist>(U &amp;&amp;other) noexcept</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Pointer</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>ab3fc3f6972074b386bf623601dfcbebd</anchor>
      <arglist>(const Pointer&lt; State &gt; &amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Pointer</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>a7cf60434462a0199b5bda1ecf4f80c01</anchor>
      <arglist>(Pointer&lt; State &gt; &amp;&amp;other) noexcept</arglist>
    </member>
    <member kind="function">
      <type>Pointer&lt; State &gt; &amp;</type>
      <name>operator=</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>af50cfd42d6d0cbdd0909f66b7d99c8db</anchor>
      <arglist>(const Pointer&lt; State &gt; &amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type>Pointer&lt; State &gt; &amp;</type>
      <name>operator=</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>ac18ee6f61f98e309e478725c61079a75</anchor>
      <arglist>(Pointer&lt; State &gt; &amp;&amp;other) noexcept</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>operator U</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>a26d98d69b279f2618edf0785882e01e5</anchor>
      <arglist>() &amp;&amp;</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator==</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>a5b3089b6c7e3e57b3ebc712b3a1b65a5</anchor>
      <arglist>(std::nullptr_t) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator!=</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>aded8d11d6271891c1fa62572e59bc6dd</anchor>
      <arglist>(std::nullptr_t) const</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~Pointer</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>a3e4509891d92dde6088f5a73b0881a64</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>operator bool</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>a6f4df0db94fe387105c56d0691a25ac4</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>State *</type>
      <name>get</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>a341521cc065c95861ed82d85e26a33d1</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const State *</type>
      <name>get</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>a98b642e46b44b5a4d10a36e216470783</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>State *</type>
      <name>operator-&gt;</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>a52a73243270252c6f91851b377958e62</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const State *</type>
      <name>operator-&gt;</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>ae225b88b0d07991e9fff931bfab2f5d9</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>State &amp;</type>
      <name>operator *</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>a047a241d186b89fe086929927e38b50a</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const State &amp;</type>
      <name>operator *</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>ab04fa2ee84e6d3c6896b715c95d36a0d</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>reset</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>ac4150a4c7336f33d3e0bdbec832154ab</anchor>
      <arglist>(State *pointer=nullptr)</arglist>
    </member>
    <member kind="function">
      <type>State &amp;</type>
      <name>emplace</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>a76f18ffe7324c11f56cf073ca2737717</anchor>
      <arglist>(Args &amp;&amp;... args)</arglist>
    </member>
    <member kind="function">
      <type>State *</type>
      <name>release</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>aaa8b2ed64ca7afb46c614e7aea7893a9</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator==</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>af9b9640eac6d17650d0cc5111c4a19b6</anchor>
      <arglist>(std::nullptr_t, const Pointer&lt; State &gt; &amp;b)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator!=</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>a141814794f6072316c8fa72afc8c7f76</anchor>
      <arglist>(std::nullptr_t, const Pointer&lt; State &gt; &amp;b)</arglist>
    </member>
    <member kind="function">
      <type>Pointer&lt; State &gt;</type>
      <name>pointer</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>a76b1a48222910763fdc5046b6625090e</anchor>
      <arglist>(State *pointer)</arglist>
    </member>
    <member kind="function">
      <type>auto</type>
      <name>pointer</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>a26b8a841d97121802988aff08bcbb0c6</anchor>
      <arglist>(State &amp;&amp;other) -&gt; decltype(Implementation::DeducedPointerConverter&lt; State &gt;::from(std::move(other)))</arglist>
    </member>
    <member kind="function">
      <type>Pointer&lt; State &gt;</type>
      <name>pointer</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>ab287dc10fa2591e662d9a9ea611a3dad</anchor>
      <arglist>(Args &amp;&amp;... args)</arglist>
    </member>
    <member kind="function">
      <type>Pointer&lt; U &gt;</type>
      <name>pointerCast</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>ae1b70d106c3a13b76188c0f7f956cb0c</anchor>
      <arglist>(Pointer&lt; State &gt; &amp;&amp;pointer)</arglist>
    </member>
    <member kind="function">
      <type>Utility::Debug &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>a9c784a034b942686ae70ce8f5b237e26</anchor>
      <arglist>(Utility::Debug &amp;debug, const Pointer&lt; State &gt; &amp;value)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>Pointer&lt; TesterState &gt;</name>
    <filename>classCorrade_1_1Containers_1_1Pointer.html</filename>
    <member kind="function">
      <type></type>
      <name>Pointer</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>adfca41b457c65ad6f7a20e4da25a551d</anchor>
      <arglist>(std::nullptr_t=nullptr) noexcept</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Pointer</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>a6961c1d838ce004acce16785ec434c91</anchor>
      <arglist>(TesterState *pointer) noexcept</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Pointer</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>afa90d4aa96c05da96180f20e3afda237</anchor>
      <arglist>(InPlaceInitT, Args &amp;&amp;... args)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Pointer</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>a30e35c7e582758f8814c2f6b6801dce8</anchor>
      <arglist>(Pointer&lt; U &gt; &amp;&amp;other) noexcept</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Pointer</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>a4e598da5a54d78e0f6e3802a3678ccde</anchor>
      <arglist>(U &amp;&amp;other) noexcept</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Pointer</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>ab3fc3f6972074b386bf623601dfcbebd</anchor>
      <arglist>(const Pointer&lt; TesterState &gt; &amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Pointer</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>a7cf60434462a0199b5bda1ecf4f80c01</anchor>
      <arglist>(Pointer&lt; TesterState &gt; &amp;&amp;other) noexcept</arglist>
    </member>
    <member kind="function">
      <type>Pointer&lt; TesterState &gt; &amp;</type>
      <name>operator=</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>af50cfd42d6d0cbdd0909f66b7d99c8db</anchor>
      <arglist>(const Pointer&lt; TesterState &gt; &amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type>Pointer&lt; TesterState &gt; &amp;</type>
      <name>operator=</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>ac18ee6f61f98e309e478725c61079a75</anchor>
      <arglist>(Pointer&lt; TesterState &gt; &amp;&amp;other) noexcept</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>operator U</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>a26d98d69b279f2618edf0785882e01e5</anchor>
      <arglist>() &amp;&amp;</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator==</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>a5b3089b6c7e3e57b3ebc712b3a1b65a5</anchor>
      <arglist>(std::nullptr_t) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator!=</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>aded8d11d6271891c1fa62572e59bc6dd</anchor>
      <arglist>(std::nullptr_t) const</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~Pointer</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>a3e4509891d92dde6088f5a73b0881a64</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>operator bool</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>a6f4df0db94fe387105c56d0691a25ac4</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>TesterState *</type>
      <name>get</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>a341521cc065c95861ed82d85e26a33d1</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const TesterState *</type>
      <name>get</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>a98b642e46b44b5a4d10a36e216470783</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>TesterState *</type>
      <name>operator-&gt;</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>a52a73243270252c6f91851b377958e62</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const TesterState *</type>
      <name>operator-&gt;</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>ae225b88b0d07991e9fff931bfab2f5d9</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>TesterState &amp;</type>
      <name>operator *</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>a047a241d186b89fe086929927e38b50a</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>const TesterState &amp;</type>
      <name>operator *</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>ab04fa2ee84e6d3c6896b715c95d36a0d</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>reset</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>ac4150a4c7336f33d3e0bdbec832154ab</anchor>
      <arglist>(TesterState *pointer=nullptr)</arglist>
    </member>
    <member kind="function">
      <type>TesterState &amp;</type>
      <name>emplace</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>a76f18ffe7324c11f56cf073ca2737717</anchor>
      <arglist>(Args &amp;&amp;... args)</arglist>
    </member>
    <member kind="function">
      <type>TesterState *</type>
      <name>release</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>aaa8b2ed64ca7afb46c614e7aea7893a9</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator==</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>af9b9640eac6d17650d0cc5111c4a19b6</anchor>
      <arglist>(std::nullptr_t, const Pointer&lt; TesterState &gt; &amp;b)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator!=</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>a141814794f6072316c8fa72afc8c7f76</anchor>
      <arglist>(std::nullptr_t, const Pointer&lt; TesterState &gt; &amp;b)</arglist>
    </member>
    <member kind="function">
      <type>Pointer&lt; TesterState &gt;</type>
      <name>pointer</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>a76b1a48222910763fdc5046b6625090e</anchor>
      <arglist>(TesterState *pointer)</arglist>
    </member>
    <member kind="function">
      <type>auto</type>
      <name>pointer</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>a26b8a841d97121802988aff08bcbb0c6</anchor>
      <arglist>(TesterState &amp;&amp;other) -&gt; decltype(Implementation::DeducedPointerConverter&lt; TesterState &gt;::from(std::move(other)))</arglist>
    </member>
    <member kind="function">
      <type>Pointer&lt; TesterState &gt;</type>
      <name>pointer</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>ab287dc10fa2591e662d9a9ea611a3dad</anchor>
      <arglist>(Args &amp;&amp;... args)</arglist>
    </member>
    <member kind="function">
      <type>Pointer&lt; U &gt;</type>
      <name>pointerCast</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>ae1b70d106c3a13b76188c0f7f956cb0c</anchor>
      <arglist>(Pointer&lt; TesterState &gt; &amp;&amp;pointer)</arglist>
    </member>
    <member kind="function">
      <type>Utility::Debug &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>classCorrade_1_1Containers_1_1Pointer.html</anchorfile>
      <anchor>a9c784a034b942686ae70ce8f5b237e26</anchor>
      <arglist>(Utility::Debug &amp;debug, const Pointer&lt; TesterState &gt; &amp;value)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>Reference&lt; Corrade::Interconnect::Emitter &gt;</name>
    <filename>classCorrade_1_1Containers_1_1Reference.html</filename>
    <member kind="function">
      <type>constexpr</type>
      <name>Reference</name>
      <anchorfile>classCorrade_1_1Containers_1_1Reference.html</anchorfile>
      <anchor>a0421d566b189294de33306844e96d9f7</anchor>
      <arglist>(Corrade::Interconnect::Emitter &amp;reference) noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>Reference</name>
      <anchorfile>classCorrade_1_1Containers_1_1Reference.html</anchorfile>
      <anchor>a4e6818b1ac252c831d5198b8e8e115a4</anchor>
      <arglist>(U other) noexcept</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>Reference</name>
      <anchorfile>classCorrade_1_1Containers_1_1Reference.html</anchorfile>
      <anchor>abf5b9c0a0a35bdf6328b3658081d40ae</anchor>
      <arglist>(Corrade::Interconnect::Emitter &amp;&amp;)=delete</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>Reference</name>
      <anchorfile>classCorrade_1_1Containers_1_1Reference.html</anchorfile>
      <anchor>af8d23ae013554277f6b59ff891df756d</anchor>
      <arglist>(Reference&lt; U &gt; other) noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>operator U</name>
      <anchorfile>classCorrade_1_1Containers_1_1Reference.html</anchorfile>
      <anchor>a752abdaca2a9f9e76bf884aad6703b29</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>operator Corrade::Interconnect::Emitter &amp;</name>
      <anchorfile>classCorrade_1_1Containers_1_1Reference.html</anchorfile>
      <anchor>a886648e43b21bfc39106afba79daf022</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>operator Reference&lt; const Corrade::Interconnect::Emitter &gt;</name>
      <anchorfile>classCorrade_1_1Containers_1_1Reference.html</anchorfile>
      <anchor>ac2fd386035105f40e6db5ef640f28e5d</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr Corrade::Interconnect::Emitter &amp;</type>
      <name>get</name>
      <anchorfile>classCorrade_1_1Containers_1_1Reference.html</anchorfile>
      <anchor>a8891dbfbf35e37be7cd08432dd5bb2a7</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr Corrade::Interconnect::Emitter *</type>
      <name>operator-&gt;</name>
      <anchorfile>classCorrade_1_1Containers_1_1Reference.html</anchorfile>
      <anchor>aeb149ea11dc647219e66f4facf98acae</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr Corrade::Interconnect::Emitter &amp;</type>
      <name>operator *</name>
      <anchorfile>classCorrade_1_1Containers_1_1Reference.html</anchorfile>
      <anchor>a0ca220977a45ca99b8e87bc6cb5d8dd8</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>Utility::Debug &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>classCorrade_1_1Containers_1_1Reference.html</anchorfile>
      <anchor>aaefc7f623d9f0c2d68e4f42e79e4b161</anchor>
      <arglist>(Utility::Debug &amp;debug, const Reference&lt; Corrade::Interconnect::Emitter &gt; &amp;value)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>StridedDimensions&lt; dimensions, std::ptrdiff_t &gt;</name>
    <filename>classCorrade_1_1Containers_1_1StridedDimensions.html</filename>
    <member kind="function">
      <type>constexpr</type>
      <name>StridedDimensions</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedDimensions.html</anchorfile>
      <anchor>a96cb09a101b6639009bfe79cdd9204b4</anchor>
      <arglist>() noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>StridedDimensions</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedDimensions.html</anchorfile>
      <anchor>a9c671335304d3df36b791cf9dfa75ac8</anchor>
      <arglist>(ValueInitT) noexcept</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>StridedDimensions</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedDimensions.html</anchorfile>
      <anchor>a1e59b07b34e98084ec6808d34a58a88f</anchor>
      <arglist>(NoInitT) noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>StridedDimensions</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedDimensions.html</anchorfile>
      <anchor>a50802cad6c511e2cf76e00b0189afab8</anchor>
      <arglist>(std::ptrdiff_t first, Args... next) noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>StridedDimensions</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedDimensions.html</anchorfile>
      <anchor>a484f60e4b07ae0e51e8c5856c6118322</anchor>
      <arglist>(StaticArrayView&lt; dimensions, const std::ptrdiff_t &gt; values) noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>StridedDimensions</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedDimensions.html</anchorfile>
      <anchor>a19e3e994e96fe16d0853dcf7453495c6</anchor>
      <arglist>(const std::ptrdiff_t(&amp;values)[dimensions]) noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>operator StaticArrayView&lt; dimensions, const std::ptrdiff_t &gt;</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedDimensions.html</anchorfile>
      <anchor>a25ca4a3bc4108137283474e5a1b2b67c</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>operator std::ptrdiff_t</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedDimensions.html</anchorfile>
      <anchor>a2451492c97bd37ae9a76ef14a986eb14</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator==</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedDimensions.html</anchorfile>
      <anchor>a7e5c4ebf3c1b92f7e30db2c2a596579b</anchor>
      <arglist>(const StridedDimensions&lt; dimensions, std::ptrdiff_t &gt; &amp;other) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator!=</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedDimensions.html</anchorfile>
      <anchor>a09d527e5798d5e827ac8b7ca3b5def82</anchor>
      <arglist>(const StridedDimensions&lt; dimensions, std::ptrdiff_t &gt; &amp;other) const</arglist>
    </member>
    <member kind="function">
      <type>constexpr std::ptrdiff_t</type>
      <name>operator[]</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedDimensions.html</anchorfile>
      <anchor>a4187e63cd7612bb4dfe370ae7cafda68</anchor>
      <arglist>(std::size_t i) const</arglist>
    </member>
    <member kind="function">
      <type>std::ptrdiff_t &amp;</type>
      <name>operator[]</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedDimensions.html</anchorfile>
      <anchor>a8345d58bb4e8920650421d8de4e7935a</anchor>
      <arglist>(std::size_t i)</arglist>
    </member>
    <member kind="function">
      <type>constexpr const std::ptrdiff_t *</type>
      <name>begin</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedDimensions.html</anchorfile>
      <anchor>a4a517253fa3136144eb9de7ab49e4278</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr const std::ptrdiff_t *</type>
      <name>cbegin</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedDimensions.html</anchorfile>
      <anchor>a59f36423a9202d3f5936ab3fc68fa6be</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr const std::ptrdiff_t *</type>
      <name>end</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedDimensions.html</anchorfile>
      <anchor>acdaf28ed3bcbc4b04b4277e457f3464a</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr const std::ptrdiff_t *</type>
      <name>cend</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedDimensions.html</anchorfile>
      <anchor>a8d5d726c6aa1574a3798787879944a72</anchor>
      <arglist>() const</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>StridedDimensions&lt; dimensions, std::size_t &gt;</name>
    <filename>classCorrade_1_1Containers_1_1StridedDimensions.html</filename>
    <member kind="function">
      <type>constexpr</type>
      <name>StridedDimensions</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedDimensions.html</anchorfile>
      <anchor>a96cb09a101b6639009bfe79cdd9204b4</anchor>
      <arglist>() noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>StridedDimensions</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedDimensions.html</anchorfile>
      <anchor>a9c671335304d3df36b791cf9dfa75ac8</anchor>
      <arglist>(ValueInitT) noexcept</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>StridedDimensions</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedDimensions.html</anchorfile>
      <anchor>a1e59b07b34e98084ec6808d34a58a88f</anchor>
      <arglist>(NoInitT) noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>StridedDimensions</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedDimensions.html</anchorfile>
      <anchor>a50802cad6c511e2cf76e00b0189afab8</anchor>
      <arglist>(std::size_t first, Args... next) noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>StridedDimensions</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedDimensions.html</anchorfile>
      <anchor>a484f60e4b07ae0e51e8c5856c6118322</anchor>
      <arglist>(StaticArrayView&lt; dimensions, const std::size_t &gt; values) noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>StridedDimensions</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedDimensions.html</anchorfile>
      <anchor>a19e3e994e96fe16d0853dcf7453495c6</anchor>
      <arglist>(const std::size_t(&amp;values)[dimensions]) noexcept</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>operator StaticArrayView&lt; dimensions, const std::size_t &gt;</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedDimensions.html</anchorfile>
      <anchor>a25ca4a3bc4108137283474e5a1b2b67c</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr</type>
      <name>operator std::size_t</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedDimensions.html</anchorfile>
      <anchor>a2451492c97bd37ae9a76ef14a986eb14</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator==</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedDimensions.html</anchorfile>
      <anchor>a7e5c4ebf3c1b92f7e30db2c2a596579b</anchor>
      <arglist>(const StridedDimensions&lt; dimensions, std::size_t &gt; &amp;other) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator!=</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedDimensions.html</anchorfile>
      <anchor>a09d527e5798d5e827ac8b7ca3b5def82</anchor>
      <arglist>(const StridedDimensions&lt; dimensions, std::size_t &gt; &amp;other) const</arglist>
    </member>
    <member kind="function">
      <type>constexpr std::size_t</type>
      <name>operator[]</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedDimensions.html</anchorfile>
      <anchor>a4187e63cd7612bb4dfe370ae7cafda68</anchor>
      <arglist>(std::size_t i) const</arglist>
    </member>
    <member kind="function">
      <type>std::size_t &amp;</type>
      <name>operator[]</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedDimensions.html</anchorfile>
      <anchor>a8345d58bb4e8920650421d8de4e7935a</anchor>
      <arglist>(std::size_t i)</arglist>
    </member>
    <member kind="function">
      <type>constexpr const std::size_t *</type>
      <name>begin</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedDimensions.html</anchorfile>
      <anchor>a4a517253fa3136144eb9de7ab49e4278</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr const std::size_t *</type>
      <name>cbegin</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedDimensions.html</anchorfile>
      <anchor>a59f36423a9202d3f5936ab3fc68fa6be</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr const std::size_t *</type>
      <name>end</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedDimensions.html</anchorfile>
      <anchor>acdaf28ed3bcbc4b04b4277e457f3464a</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>constexpr const std::size_t *</type>
      <name>cend</name>
      <anchorfile>classCorrade_1_1Containers_1_1StridedDimensions.html</anchorfile>
      <anchor>a8d5d726c6aa1574a3798787879944a72</anchor>
      <arglist>() const</arglist>
    </member>
  </compound>
  <compound kind="namespace">
    <name>Corrade</name>
    <filename>namespaceCorrade.html</filename>
    <namespace>Corrade::Containers</namespace>
    <namespace>Corrade::Interconnect</namespace>
    <namespace>Corrade::PluginManager</namespace>
    <namespace>Corrade::TestSuite</namespace>
    <namespace>Corrade::Utility</namespace>
  </compound>
  <compound kind="namespace">
    <name>Corrade::Containers</name>
    <filename>namespaceCorrade_1_1Containers.html</filename>
    <class kind="class">Corrade::Containers::Array</class>
    <class kind="class">Corrade::Containers::ArrayView</class>
    <class kind="class">Corrade::Containers::ArrayView&lt; const void &gt;</class>
    <class kind="class">Corrade::Containers::ArrayView&lt; void &gt;</class>
    <class kind="struct">Corrade::Containers::DefaultInitT</class>
    <class kind="struct">Corrade::Containers::DirectInitT</class>
    <class kind="class">Corrade::Containers::EnumSet</class>
    <class kind="struct">Corrade::Containers::InPlaceInitT</class>
    <class kind="class">Corrade::Containers::LinkedList</class>
    <class kind="class">Corrade::Containers::LinkedListItem</class>
    <class kind="struct">Corrade::Containers::NoCreateT</class>
    <class kind="struct">Corrade::Containers::NoInitT</class>
    <class kind="struct">Corrade::Containers::NullOptT</class>
    <class kind="class">Corrade::Containers::Optional</class>
    <class kind="class">Corrade::Containers::Pointer</class>
    <class kind="class">Corrade::Containers::Reference</class>
    <class kind="class">Corrade::Containers::ScopeGuard</class>
    <class kind="class">Corrade::Containers::StaticArray</class>
    <class kind="class">Corrade::Containers::StaticArrayView</class>
    <class kind="class">Corrade::Containers::StridedArrayView</class>
    <class kind="class">Corrade::Containers::StridedDimensions</class>
    <class kind="class">Corrade::Containers::StridedIterator</class>
    <class kind="struct">Corrade::Containers::ValueInitT</class>
    <member kind="typedef">
      <type>ScopeGuard</type>
      <name>ScopedExit</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>a632d2dde79e960476ba9d9aae7f6f4c6</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>StridedArrayView&lt; 1, T &gt;</type>
      <name>StridedArrayView1D</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>a161bca60e6aa4f6f998b6f3cfa64eaef</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>StridedArrayView&lt; 2, T &gt;</type>
      <name>StridedArrayView2D</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>aad0779dab68e1061b22da995439f668b</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>StridedArrayView&lt; 3, T &gt;</type>
      <name>StridedArrayView3D</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>a8fd4827befab8d70f0da5e4300bfdc21</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>StridedArrayView&lt; 4, T &gt;</type>
      <name>StridedArrayView4D</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>ac01ba7473315887995a8680b259c9c74</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type>ArrayView&lt; T &gt;</type>
      <name>arrayView</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>a57f8d4e0a25951f539461539c90d4e25</anchor>
      <arglist>(Array&lt; T, D &gt; &amp;array)</arglist>
    </member>
    <member kind="function">
      <type>ArrayView&lt; const T &gt;</type>
      <name>arrayView</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>a4258433529f847d3e6ed10442f6b9b8b</anchor>
      <arglist>(const Array&lt; T, D &gt; &amp;array)</arglist>
    </member>
    <member kind="function">
      <type>ArrayView&lt; U &gt;</type>
      <name>arrayCast</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>ad5f0615634b2553b0cb304aa58da3347</anchor>
      <arglist>(Array&lt; T, D &gt; &amp;array)</arglist>
    </member>
    <member kind="function">
      <type>ArrayView&lt; const U &gt;</type>
      <name>arrayCast</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>a9a1ca1efe52f9930f1b31b9feab5a145</anchor>
      <arglist>(const Array&lt; T, D &gt; &amp;array)</arglist>
    </member>
    <member kind="function">
      <type>std::size_t</type>
      <name>arraySize</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>af8a4e6b17dca34a5eb82b6a8da8c398b</anchor>
      <arglist>(const Array&lt; T &gt; &amp;view)</arglist>
    </member>
    <member kind="function">
      <type>constexpr ArrayView&lt; T &gt;</type>
      <name>arrayView</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>a32ef8daa386aee742e5f39fc5b1516fd</anchor>
      <arglist>(T *data, std::size_t size)</arglist>
    </member>
    <member kind="function">
      <type>constexpr ArrayView&lt; T &gt;</type>
      <name>arrayView</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>ac8edb9cd0a74f5a0da2ef9c6d03f9ef2</anchor>
      <arglist>(T(&amp;data)[size])</arglist>
    </member>
    <member kind="function">
      <type>constexpr ArrayView&lt; T &gt;</type>
      <name>arrayView</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>ac450a49515ff20058552c732542b1b13</anchor>
      <arglist>(StaticArrayView&lt; size, T &gt; view)</arglist>
    </member>
    <member kind="function">
      <type>constexpr ArrayView&lt; T &gt;</type>
      <name>arrayView</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>a3a43320e58651eafa9774e04c68545e5</anchor>
      <arglist>(ArrayView&lt; T &gt; view)</arglist>
    </member>
    <member kind="function">
      <type>constexpr U</type>
      <name>arrayView</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>ade5f07807e9559c80c45e3c314299b91</anchor>
      <arglist>(T &amp;&amp;other)</arglist>
    </member>
    <member kind="function">
      <type>ArrayView&lt; U &gt;</type>
      <name>arrayCast</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>a8e6435208f2cda38505084a930a3e9ec</anchor>
      <arglist>(ArrayView&lt; T &gt; view)</arglist>
    </member>
    <member kind="function">
      <type>constexpr std::size_t</type>
      <name>arraySize</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>a19e087600ec3a4f81931386ee28e39ef</anchor>
      <arglist>(ArrayView&lt; T &gt; view)</arglist>
    </member>
    <member kind="function">
      <type>constexpr std::size_t</type>
      <name>arraySize</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>a9e4a13e421a34a19eec002ed65bda582</anchor>
      <arglist>(StaticArrayView&lt; size_, T &gt;)</arglist>
    </member>
    <member kind="function">
      <type>constexpr std::size_t</type>
      <name>arraySize</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>aaf76e645cbe62b360a0f13de4dff2837</anchor>
      <arglist>(T(&amp;)[size_])</arglist>
    </member>
    <member kind="function">
      <type>constexpr StaticArrayView&lt; size, T &gt;</type>
      <name>staticArrayView</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>a58d7bb8d46f44949b57b226485db4a9b</anchor>
      <arglist>(T *data)</arglist>
    </member>
    <member kind="function">
      <type>constexpr StaticArrayView&lt; size, T &gt;</type>
      <name>staticArrayView</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>a3cf428b256a5ed8941145e4d15ae8b45</anchor>
      <arglist>(T(&amp;data)[size])</arglist>
    </member>
    <member kind="function">
      <type>constexpr StaticArrayView&lt; size, T &gt;</type>
      <name>staticArrayView</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>aeb96550df22942b8379e77d3942c88c4</anchor>
      <arglist>(StaticArrayView&lt; size, T &gt; view)</arglist>
    </member>
    <member kind="function">
      <type>constexpr U</type>
      <name>staticArrayView</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>a2f1123798b8bee4ebadf785646108c98</anchor>
      <arglist>(T &amp;&amp;other)</arglist>
    </member>
    <member kind="function">
      <type>StaticArrayView&lt; size *sizeof(T)/sizeof(U), U &gt;</type>
      <name>arrayCast</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>aee41b58bdb45963c476ce7554dda2290</anchor>
      <arglist>(StaticArrayView&lt; size, T &gt; view)</arglist>
    </member>
    <member kind="function">
      <type>StaticArrayView&lt; size *sizeof(T)/sizeof(U), U &gt;</type>
      <name>arrayCast</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>aee9c43d476128f05403e638ce539c90d</anchor>
      <arglist>(T(&amp;data)[size])</arglist>
    </member>
    <member kind="function">
      <type>Utility::Debug &amp;</type>
      <name>enumSetDebugOutput</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>a12c4ea794fa62c56969b1311e2b5c21b</anchor>
      <arglist>(Utility::Debug &amp;debug, EnumSet&lt; T, fullValue &gt; value, const char *empty, std::initializer_list&lt; T &gt; enums)</arglist>
    </member>
    <member kind="function">
      <type>Optional&lt; typename std::decay&lt; T &gt;::type &gt;</type>
      <name>optional</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>a71e22f3640eb28dd7b2f7b8339931224</anchor>
      <arglist>(T &amp;&amp;value)</arglist>
    </member>
    <member kind="function">
      <type>Optional&lt; T &gt;</type>
      <name>optional</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>a828d28109eb44c02f39b554a392606e9</anchor>
      <arglist>(Args &amp;&amp;... args)</arglist>
    </member>
    <member kind="function">
      <type>auto</type>
      <name>optional</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>ab0ccf001156df9d35c557ce79a66b927</anchor>
      <arglist>(T &amp;&amp;other) -&gt; decltype(Implementation::DeducedOptionalConverter&lt; typename std::decay&lt; T &gt;::type &gt;::from(std::forward&lt; T &gt;(other)))</arglist>
    </member>
    <member kind="function">
      <type>Pointer&lt; T &gt;</type>
      <name>pointer</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>a76b1a48222910763fdc5046b6625090e</anchor>
      <arglist>(T *pointer)</arglist>
    </member>
    <member kind="function">
      <type>auto</type>
      <name>pointer</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>a26b8a841d97121802988aff08bcbb0c6</anchor>
      <arglist>(T &amp;&amp;other) -&gt; decltype(Implementation::DeducedPointerConverter&lt; T &gt;::from(std::move(other)))</arglist>
    </member>
    <member kind="function">
      <type>Pointer&lt; U &gt;</type>
      <name>pointerCast</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>ae1b70d106c3a13b76188c0f7f956cb0c</anchor>
      <arglist>(Pointer&lt; T &gt; &amp;&amp;pointer)</arglist>
    </member>
    <member kind="function">
      <type>Pointer&lt; T &gt;</type>
      <name>pointer</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>ab287dc10fa2591e662d9a9ea611a3dad</anchor>
      <arglist>(Args &amp;&amp;... args)</arglist>
    </member>
    <member kind="function">
      <type>constexpr ArrayView&lt; T &gt;</type>
      <name>arrayView</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>aa8111199cdb6cc30c470e3cc9c54a30e</anchor>
      <arglist>(StaticArray&lt; size, T &gt; &amp;array)</arglist>
    </member>
    <member kind="function">
      <type>constexpr ArrayView&lt; const T &gt;</type>
      <name>arrayView</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>a5b40a450ade4927381bf97a165e3621e</anchor>
      <arglist>(const StaticArray&lt; size, T &gt; &amp;array)</arglist>
    </member>
    <member kind="function">
      <type>constexpr StaticArrayView&lt; size, T &gt;</type>
      <name>staticArrayView</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>a4f6c86de70a1ab496a1d798648c22f80</anchor>
      <arglist>(StaticArray&lt; size, T &gt; &amp;array)</arglist>
    </member>
    <member kind="function">
      <type>constexpr StaticArrayView&lt; size, const T &gt;</type>
      <name>staticArrayView</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>ab15dc352f75f85e562eaa85ae058ac9f</anchor>
      <arglist>(const StaticArray&lt; size, T &gt; &amp;array)</arglist>
    </member>
    <member kind="function">
      <type>StaticArrayView&lt; size *sizeof(T)/sizeof(U), U &gt;</type>
      <name>arrayCast</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>a5ebf447d7f388e81fe6d34910c80cfbe</anchor>
      <arglist>(StaticArray&lt; size, T &gt; &amp;array)</arglist>
    </member>
    <member kind="function">
      <type>StaticArrayView&lt; size *sizeof(T)/sizeof(U), const U &gt;</type>
      <name>arrayCast</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>aae0b8c37cdca03924fac3a3ff457bf9b</anchor>
      <arglist>(const StaticArray&lt; size, T &gt; &amp;array)</arglist>
    </member>
    <member kind="function">
      <type>constexpr std::size_t</type>
      <name>arraySize</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>a0cad6e09b65666040bb2460df883e4a3</anchor>
      <arglist>(const StaticArray&lt; size_, T &gt; &amp;)</arglist>
    </member>
    <member kind="function">
      <type>constexpr StridedArrayView1D&lt; T &gt;</type>
      <name>stridedArrayView</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>a28f0f3e4514d0a6d4d463f7bd2884f16</anchor>
      <arglist>(T(&amp;data)[size])</arglist>
    </member>
    <member kind="function">
      <type>constexpr StridedArrayView1D&lt; T &gt;</type>
      <name>stridedArrayView</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>a8a18ea77b75e5838af7828f4b681de1d</anchor>
      <arglist>(ArrayView&lt; T &gt; view)</arglist>
    </member>
    <member kind="function">
      <type>constexpr StridedArrayView1D&lt; T &gt;</type>
      <name>stridedArrayView</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>a5687e5c332876211a23a2202ea89231b</anchor>
      <arglist>(StaticArrayView&lt; size, T &gt; view)</arglist>
    </member>
    <member kind="function">
      <type>constexpr StridedArrayView&lt; dimensions, T &gt;</type>
      <name>stridedArrayView</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>ab15f7dff3df81f6529c2b9e401f813dc</anchor>
      <arglist>(StridedArrayView&lt; dimensions, T &gt; view)</arglist>
    </member>
    <member kind="function">
      <type>constexpr U</type>
      <name>stridedArrayView</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>af0b91e302ea8ee6b0c76fe99fe6af1f8</anchor>
      <arglist>(T &amp;&amp;other)</arglist>
    </member>
    <member kind="function">
      <type>StridedArrayView&lt; dimensions, U &gt;</type>
      <name>arrayCast</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>a529bc0bc79d6272d1e53c85c5ecd7005</anchor>
      <arglist>(const StridedArrayView&lt; dimensions, T &gt; &amp;view)</arglist>
    </member>
    <member kind="function">
      <type>StridedArrayView&lt; newDimensions, U &gt;</type>
      <name>arrayCast</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>abb1caf4042587dc306fc92204c441feb</anchor>
      <arglist>(const StridedArrayView&lt; dimensions, T &gt; &amp;view)</arglist>
    </member>
    <member kind="variable">
      <type>constexpr NullOptT</type>
      <name>NullOpt</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>a047ff43a43f3841e3c7125e1cc41bb60</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>constexpr DefaultInitT</type>
      <name>DefaultInit</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>a10f8c95aaa0d51bb77976fe3e4924a5e</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>constexpr ValueInitT</type>
      <name>ValueInit</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>a26d49cbb15d20d7dc72878f522e24444</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>constexpr NoInitT</type>
      <name>NoInit</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>a7094104336b357363773b3f29891d048</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>constexpr NoCreateT</type>
      <name>NoCreate</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>a4626e472e29e045795dc8d55909b3e3f</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>constexpr DirectInitT</type>
      <name>DirectInit</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>a69da0a827c69b45cec539ec0c201a119</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>constexpr InPlaceInitT</type>
      <name>InPlaceInit</name>
      <anchorfile>namespaceCorrade_1_1Containers.html</anchorfile>
      <anchor>ab86b6e81b5157279ebfbf2b716fb2d6d</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="namespace">
    <name>Corrade::Interconnect</name>
    <filename>namespaceCorrade_1_1Interconnect.html</filename>
    <class kind="class">Corrade::Interconnect::Connection</class>
    <class kind="class">Corrade::Interconnect::Emitter</class>
    <class kind="class">Corrade::Interconnect::Receiver</class>
    <class kind="class">Corrade::Interconnect::StateMachine</class>
    <class kind="class">Corrade::Interconnect::StateTransition</class>
    <member kind="function">
      <type>bool</type>
      <name>disconnect</name>
      <anchorfile>namespaceCorrade_1_1Interconnect.html</anchorfile>
      <anchor>a08518bbf1547ae48fbd32afeba6fc787</anchor>
      <arglist>(Emitter &amp;emitter, const Connection &amp;connection)</arglist>
    </member>
    <member kind="function">
      <type>Connection</type>
      <name>connect</name>
      <anchorfile>namespaceCorrade_1_1Interconnect.html</anchorfile>
      <anchor>afe6ec8e2281aa5a2804094419d135005</anchor>
      <arglist>(EmitterObject &amp;emitter, Interconnect::Emitter::Signal(Emitter::*signal)(Args...), Functor &amp;&amp;slot)</arglist>
    </member>
    <member kind="function">
      <type>Connection</type>
      <name>connect</name>
      <anchorfile>namespaceCorrade_1_1Interconnect.html</anchorfile>
      <anchor>a52f95755ab08bad47be07955f641f2cc</anchor>
      <arglist>(EmitterObject &amp;emitter, Interconnect::Emitter::Signal(Emitter::*signal)(Args...), ReceiverObject &amp;receiver, void(Receiver::*slot)(Args...))</arglist>
    </member>
  </compound>
  <compound kind="namespace">
    <name>Corrade::PluginManager</name>
    <filename>namespaceCorrade_1_1PluginManager.html</filename>
    <class kind="class">Corrade::PluginManager::AbstractManager</class>
    <class kind="class">Corrade::PluginManager::AbstractManagingPlugin</class>
    <class kind="class">Corrade::PluginManager::AbstractPlugin</class>
    <class kind="class">Corrade::PluginManager::Manager</class>
    <class kind="class">Corrade::PluginManager::PluginMetadata</class>
    <member kind="typedef">
      <type>Containers::EnumSet&lt; LoadState &gt;</type>
      <name>LoadStates</name>
      <anchorfile>namespaceCorrade_1_1PluginManager.html</anchorfile>
      <anchor>a61b14635bc0c147c65e8be80260ca891</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumeration">
      <type></type>
      <name>LoadState</name>
      <anchorfile>namespaceCorrade_1_1PluginManager.html</anchorfile>
      <anchor>aa240e935222a178e1acb5c0853f15547</anchor>
      <arglist></arglist>
      <enumvalue file="namespaceCorrade_1_1PluginManager.html" anchor="aa240e935222a178e1acb5c0853f15547a38c300f4fc9ce8a77aad4a30de05cad8">NotFound</enumvalue>
      <enumvalue file="namespaceCorrade_1_1PluginManager.html" anchor="aa240e935222a178e1acb5c0853f15547a3485912f892918a4ed6b357abad8e5a4">WrongPluginVersion</enumvalue>
      <enumvalue file="namespaceCorrade_1_1PluginManager.html" anchor="aa240e935222a178e1acb5c0853f15547ae90c7458dc20c6326d3a34071de85e3f">WrongInterfaceVersion</enumvalue>
      <enumvalue file="namespaceCorrade_1_1PluginManager.html" anchor="aa240e935222a178e1acb5c0853f15547ad6ab066e75221cc9a69bfede5f75347c">WrongMetadataFile</enumvalue>
      <enumvalue file="namespaceCorrade_1_1PluginManager.html" anchor="aa240e935222a178e1acb5c0853f15547a3c062ee3a4f39445ef4ba44ada5bde3c">UnresolvedDependency</enumvalue>
      <enumvalue file="namespaceCorrade_1_1PluginManager.html" anchor="aa240e935222a178e1acb5c0853f15547a3729d9667c9ed82ae96b6174b288a3a5">LoadFailed</enumvalue>
      <enumvalue file="namespaceCorrade_1_1PluginManager.html" anchor="aa240e935222a178e1acb5c0853f15547a84a8921b25f505d0d2077aeb5db4bc16">Static</enumvalue>
      <enumvalue file="namespaceCorrade_1_1PluginManager.html" anchor="aa240e935222a178e1acb5c0853f15547a7381d487d18845b379422325c0a768d6">Loaded</enumvalue>
      <enumvalue file="namespaceCorrade_1_1PluginManager.html" anchor="aa240e935222a178e1acb5c0853f15547a5111e24c1ecc6266ce0de4b4dc42033b">NotLoaded</enumvalue>
      <enumvalue file="namespaceCorrade_1_1PluginManager.html" anchor="aa240e935222a178e1acb5c0853f15547ac7de13f47f40f470d0795d224221a577">UnloadFailed</enumvalue>
      <enumvalue file="namespaceCorrade_1_1PluginManager.html" anchor="aa240e935222a178e1acb5c0853f15547ab651efdb98a5d6bd2b3935d0c3f4a5e2">Required</enumvalue>
      <enumvalue file="namespaceCorrade_1_1PluginManager.html" anchor="aa240e935222a178e1acb5c0853f15547a019d1ca7d50cc54b995f60d456435e87">Used</enumvalue>
    </member>
    <member kind="function">
      <type>Utility::Debug &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>namespaceCorrade_1_1PluginManager.html</anchorfile>
      <anchor>a73b06a0cd1333798b796a2061dc6bedd</anchor>
      <arglist>(Utility::Debug &amp;debug, PluginManager::LoadState value)</arglist>
    </member>
    <member kind="function">
      <type>Utility::Debug &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>namespaceCorrade_1_1PluginManager.html</anchorfile>
      <anchor>a75f06adcf2b3f416601f1f58507a3f15</anchor>
      <arglist>(Utility::Debug &amp;debug, PluginManager::LoadStates value)</arglist>
    </member>
  </compound>
  <compound kind="namespace">
    <name>Corrade::TestSuite</name>
    <filename>namespaceCorrade_1_1TestSuite.html</filename>
    <namespace>Corrade::TestSuite::Compare</namespace>
    <class kind="class">Corrade::TestSuite::Comparator</class>
    <class kind="class">Corrade::TestSuite::Comparator&lt; double &gt;</class>
    <class kind="class">Corrade::TestSuite::Comparator&lt; float &gt;</class>
    <class kind="class">Corrade::TestSuite::Comparator&lt; long double &gt;</class>
    <class kind="class">Corrade::TestSuite::Tester</class>
    <member kind="typedef">
      <type>Containers::EnumSet&lt; ComparisonStatusFlag &gt;</type>
      <name>ComparisonStatusFlags</name>
      <anchorfile>namespaceCorrade_1_1TestSuite.html</anchorfile>
      <anchor>a4578a3861445b1ed4bd30a160f9380bc</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumeration">
      <type></type>
      <name>ComparisonStatusFlag</name>
      <anchorfile>namespaceCorrade_1_1TestSuite.html</anchorfile>
      <anchor>a1dd98653ce2b732a42061f6cb18c7831</anchor>
      <arglist></arglist>
      <enumvalue file="namespaceCorrade_1_1TestSuite.html" anchor="a1dd98653ce2b732a42061f6cb18c7831ad7c8c85bf79bbe1b7188497c32c3b0ca">Failed</enumvalue>
      <enumvalue file="namespaceCorrade_1_1TestSuite.html" anchor="a1dd98653ce2b732a42061f6cb18c7831a0eaadb4fcb48a0a0ed7bc9868be9fbaa">Warning</enumvalue>
      <enumvalue file="namespaceCorrade_1_1TestSuite.html" anchor="a1dd98653ce2b732a42061f6cb18c7831a4c2a8fe7eaf24721cc7a9f0175115bd4">Message</enumvalue>
      <enumvalue file="namespaceCorrade_1_1TestSuite.html" anchor="a1dd98653ce2b732a42061f6cb18c7831ad4a9fa383ab700c5bdd6f31cf7df0faf">Verbose</enumvalue>
      <enumvalue file="namespaceCorrade_1_1TestSuite.html" anchor="a1dd98653ce2b732a42061f6cb18c7831a7f84beab04579bef70043ca0cc72fb85">Diagnostic</enumvalue>
      <enumvalue file="namespaceCorrade_1_1TestSuite.html" anchor="a1dd98653ce2b732a42061f6cb18c7831ad86c142bfd1ab654a16c8eaf191eade7">VerboseDiagnostic</enumvalue>
    </member>
    <member kind="function">
      <type>Utility::Debug &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>namespaceCorrade_1_1TestSuite.html</anchorfile>
      <anchor>a1c1c3434f9c7b2b2902b319c1f87e81c</anchor>
      <arglist>(Utility::Debug &amp;debug, ComparisonStatusFlag value)</arglist>
    </member>
    <member kind="function">
      <type>Utility::Debug &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>namespaceCorrade_1_1TestSuite.html</anchorfile>
      <anchor>a9f9c017468d30a59a39b718d1ff4a91e</anchor>
      <arglist>(Utility::Debug &amp;debug, ComparisonStatusFlags value)</arglist>
    </member>
  </compound>
  <compound kind="namespace">
    <name>Corrade::TestSuite::Compare</name>
    <filename>namespaceCorrade_1_1TestSuite_1_1Compare.html</filename>
    <class kind="class">Corrade::TestSuite::Compare::Around</class>
    <class kind="class">Corrade::TestSuite::Compare::Container</class>
    <class kind="class">Corrade::TestSuite::Compare::File</class>
    <class kind="class">Corrade::TestSuite::Compare::FileToString</class>
    <class kind="class">Corrade::TestSuite::Compare::Greater</class>
    <class kind="class">Corrade::TestSuite::Compare::GreaterOrEqual</class>
    <class kind="class">Corrade::TestSuite::Compare::Less</class>
    <class kind="class">Corrade::TestSuite::Compare::LessOrEqual</class>
    <class kind="class">Corrade::TestSuite::Compare::SortedContainer</class>
    <class kind="class">Corrade::TestSuite::Compare::StringToFile</class>
    <member kind="function">
      <type>Around&lt; T &gt;</type>
      <name>around</name>
      <anchorfile>namespaceCorrade_1_1TestSuite_1_1Compare.html</anchorfile>
      <anchor>a76165bdc14d6ddd8ba20356ffd11e2e5</anchor>
      <arglist>(T epsilon)</arglist>
    </member>
  </compound>
  <compound kind="namespace">
    <name>Corrade::Utility</name>
    <filename>namespaceCorrade_1_1Utility.html</filename>
    <namespace>Corrade::Utility::Directory</namespace>
    <namespace>Corrade::Utility::String</namespace>
    <namespace>Corrade::Utility::System</namespace>
    <namespace>Corrade::Utility::Unicode</namespace>
    <class kind="class">Corrade::Utility::AbstractHash</class>
    <class kind="class">Corrade::Utility::AndroidLogStreamBuffer</class>
    <class kind="class">Corrade::Utility::Arguments</class>
    <class kind="class">Corrade::Utility::Configuration</class>
    <class kind="class">Corrade::Utility::ConfigurationGroup</class>
    <class kind="struct">Corrade::Utility::ConfigurationValue</class>
    <class kind="struct">Corrade::Utility::ConfigurationValue&lt; bool &gt;</class>
    <class kind="struct">Corrade::Utility::ConfigurationValue&lt; char32_t &gt;</class>
    <class kind="struct">Corrade::Utility::ConfigurationValue&lt; double &gt;</class>
    <class kind="struct">Corrade::Utility::ConfigurationValue&lt; float &gt;</class>
    <class kind="struct">Corrade::Utility::ConfigurationValue&lt; int &gt;</class>
    <class kind="struct">Corrade::Utility::ConfigurationValue&lt; long &gt;</class>
    <class kind="struct">Corrade::Utility::ConfigurationValue&lt; long double &gt;</class>
    <class kind="struct">Corrade::Utility::ConfigurationValue&lt; long long &gt;</class>
    <class kind="struct">Corrade::Utility::ConfigurationValue&lt; short &gt;</class>
    <class kind="struct">Corrade::Utility::ConfigurationValue&lt; std::string &gt;</class>
    <class kind="struct">Corrade::Utility::ConfigurationValue&lt; unsigned int &gt;</class>
    <class kind="struct">Corrade::Utility::ConfigurationValue&lt; unsigned long &gt;</class>
    <class kind="struct">Corrade::Utility::ConfigurationValue&lt; unsigned long long &gt;</class>
    <class kind="struct">Corrade::Utility::ConfigurationValue&lt; unsigned short &gt;</class>
    <class kind="class">Corrade::Utility::Debug</class>
    <class kind="class">Corrade::Utility::Endianness</class>
    <class kind="class">Corrade::Utility::Error</class>
    <class kind="class">Corrade::Utility::Fatal</class>
    <class kind="class">Corrade::Utility::FileWatcher</class>
    <class kind="class">Corrade::Utility::HashDigest</class>
    <class kind="class">Corrade::Utility::MurmurHash2</class>
    <class kind="class">Corrade::Utility::Resource</class>
    <class kind="class">Corrade::Utility::Sha1</class>
    <class kind="class">Corrade::Utility::Tweakable</class>
    <class kind="struct">Corrade::Utility::TweakableParser</class>
    <class kind="struct">Corrade::Utility::TweakableParser&lt; bool &gt;</class>
    <class kind="struct">Corrade::Utility::TweakableParser&lt; char &gt;</class>
    <class kind="struct">Corrade::Utility::TweakableParser&lt; double &gt;</class>
    <class kind="struct">Corrade::Utility::TweakableParser&lt; float &gt;</class>
    <class kind="struct">Corrade::Utility::TweakableParser&lt; int &gt;</class>
    <class kind="struct">Corrade::Utility::TweakableParser&lt; long &gt;</class>
    <class kind="struct">Corrade::Utility::TweakableParser&lt; long double &gt;</class>
    <class kind="struct">Corrade::Utility::TweakableParser&lt; long long &gt;</class>
    <class kind="struct">Corrade::Utility::TweakableParser&lt; unsigned int &gt;</class>
    <class kind="struct">Corrade::Utility::TweakableParser&lt; unsigned long &gt;</class>
    <class kind="struct">Corrade::Utility::TweakableParser&lt; unsigned long long &gt;</class>
    <class kind="class">Corrade::Utility::Warning</class>
    <member kind="typedef">
      <type>Containers::EnumSet&lt; ConfigurationValueFlag &gt;</type>
      <name>ConfigurationValueFlags</name>
      <anchorfile>namespaceCorrade_1_1Utility.html</anchorfile>
      <anchor>a4d1891e33318b3189e012812127acd8e</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::integral_constant&lt; bool, implementation-specific &gt;</type>
      <name>IsIterable</name>
      <anchorfile>namespaceCorrade_1_1Utility.html</anchorfile>
      <anchor>a5b6f2707e54c886af00f4738ff83afa2</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::integral_constant&lt; bool, implementation-specific &gt;</type>
      <name>IsStringLike</name>
      <anchorfile>namespaceCorrade_1_1Utility.html</anchorfile>
      <anchor>a25706236e8711798719a814357f6bffd</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumeration">
      <type></type>
      <name>ConfigurationValueFlag</name>
      <anchorfile>namespaceCorrade_1_1Utility.html</anchorfile>
      <anchor>acac747502f9bde1bf73cbfbe661c780d</anchor>
      <arglist></arglist>
      <enumvalue file="namespaceCorrade_1_1Utility.html" anchor="acac747502f9bde1bf73cbfbe661c780da594be08882c8e9d5efb9eeb62f303744">Oct</enumvalue>
      <enumvalue file="namespaceCorrade_1_1Utility.html" anchor="acac747502f9bde1bf73cbfbe661c780da92640bd72988395b326c888614f8937a">Hex</enumvalue>
      <enumvalue file="namespaceCorrade_1_1Utility.html" anchor="acac747502f9bde1bf73cbfbe661c780da21234a0e100d74037a4da2e53f3200d7">Scientific</enumvalue>
      <enumvalue file="namespaceCorrade_1_1Utility.html" anchor="acac747502f9bde1bf73cbfbe661c780da621e7b8ece62fecc55e883252ff2fbe7">Uppercase</enumvalue>
    </member>
    <member kind="enumeration">
      <type></type>
      <name>TweakableState</name>
      <anchorfile>namespaceCorrade_1_1Utility.html</anchorfile>
      <anchor>aa187573406978367665d0642ec5b6b04</anchor>
      <arglist></arglist>
      <enumvalue file="namespaceCorrade_1_1Utility.html" anchor="aa187573406978367665d0642ec5b6b04a4bac8cdf0a968472b519b3b295d0d48b">NoChange</enumvalue>
      <enumvalue file="namespaceCorrade_1_1Utility.html" anchor="aa187573406978367665d0642ec5b6b04a505a83f220c02df2f85c3810cd9ceb38">Success</enumvalue>
      <enumvalue file="namespaceCorrade_1_1Utility.html" anchor="aa187573406978367665d0642ec5b6b04a780b2be805c1437cd6730ba91f6107e3">Recompile</enumvalue>
      <enumvalue file="namespaceCorrade_1_1Utility.html" anchor="aa187573406978367665d0642ec5b6b04a902b0d55fddef6f8d651fe1035b7d4bd">Error</enumvalue>
    </member>
    <member kind="function">
      <type>Debug &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>namespaceCorrade_1_1Utility.html</anchorfile>
      <anchor>ae2cee4795e84d13d29d1d9e1c805ecd2</anchor>
      <arglist>(Debug &amp;debug, const T &amp;value)</arglist>
    </member>
    <member kind="function">
      <type>Debug &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>namespaceCorrade_1_1Utility.html</anchorfile>
      <anchor>ac103e9217a8710bd03ee4783d3d6ed4b</anchor>
      <arglist>(Debug &amp;debug, const Iterable &amp;value)</arglist>
    </member>
    <member kind="function">
      <type>Debug &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>namespaceCorrade_1_1Utility.html</anchorfile>
      <anchor>a76d39b67ac31b0abf23b5c5d4bec9f15</anchor>
      <arglist>(Debug &amp;debug, const std::pair&lt; T, U &gt; &amp;value)</arglist>
    </member>
    <member kind="function">
      <type>Debug &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>namespaceCorrade_1_1Utility.html</anchorfile>
      <anchor>a516b71a8f99c9bd0edea7275889ebc7a</anchor>
      <arglist>(Debug &amp;debug, const std::string &amp;value)</arglist>
    </member>
    <member kind="function">
      <type>Debug &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>namespaceCorrade_1_1Utility.html</anchorfile>
      <anchor>af6d7c9ff8d1c4acaed2c0d5bfe8b4a63</anchor>
      <arglist>(Debug &amp;debug, const std::basic_string&lt; T &gt; &amp;value)</arglist>
    </member>
    <member kind="function">
      <type>Debug &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>namespaceCorrade_1_1Utility.html</anchorfile>
      <anchor>a9f9d27d6f040e6e490f2d73c5a0b8b56</anchor>
      <arglist>(Debug &amp;debug, const std::tuple&lt; Args... &gt; &amp;value)</arglist>
    </member>
    <member kind="function">
      <type>Containers::Array&lt; char &gt;</type>
      <name>format</name>
      <anchorfile>namespaceCorrade_1_1Utility.html</anchorfile>
      <anchor>a5b85466788fc3d71b143909dda1ef44a</anchor>
      <arglist>(const char *format, const Args &amp;... args)</arglist>
    </member>
    <member kind="function">
      <type>std::size_t</type>
      <name>formatInto</name>
      <anchorfile>namespaceCorrade_1_1Utility.html</anchorfile>
      <anchor>a4cbdd7e7dfea1e5639ee8efa76e78b53</anchor>
      <arglist>(const Containers::ArrayView&lt; char &gt; &amp;buffer, const char *format, const Args &amp;... args)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>formatInto</name>
      <anchorfile>namespaceCorrade_1_1Utility.html</anchorfile>
      <anchor>a3b0e22fcc7bd43f5758df742ba40ebb1</anchor>
      <arglist>(std::FILE *file, const char *format, const Args &amp;... args)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>print</name>
      <anchorfile>namespaceCorrade_1_1Utility.html</anchorfile>
      <anchor>ab27a2c45ff7f820fa421daae6cdb82af</anchor>
      <arglist>(const char *format, const Args &amp;... args)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>printError</name>
      <anchorfile>namespaceCorrade_1_1Utility.html</anchorfile>
      <anchor>a1d88fd05905f0c021fe6169068441102</anchor>
      <arglist>(const char *format, const Args &amp;... args)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>formatString</name>
      <anchorfile>namespaceCorrade_1_1Utility.html</anchorfile>
      <anchor>aaa50b1dc1eaffad13a65ee141a5c9e4f</anchor>
      <arglist>(const char *format, const Args &amp;... args)</arglist>
    </member>
    <member kind="function">
      <type>std::size_t</type>
      <name>formatInto</name>
      <anchorfile>namespaceCorrade_1_1Utility.html</anchorfile>
      <anchor>aaf6e2f081fa8fd67065dd7b39811f5f6</anchor>
      <arglist>(std::string &amp;string, std::size_t offset, const char *format, const Args &amp;... args)</arglist>
    </member>
    <member kind="function">
      <type>Debug &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>namespaceCorrade_1_1Utility.html</anchorfile>
      <anchor>ae0d97ad312a2030f1344b8839f5d9779</anchor>
      <arglist>(Debug &amp;debug, TweakableState value)</arglist>
    </member>
    <member kind="function">
      <type>To</type>
      <name>bitCast</name>
      <anchorfile>namespaceCorrade_1_1Utility.html</anchorfile>
      <anchor>a3831cece822dfde455336664676f6867</anchor>
      <arglist>(const From &amp;from)</arglist>
    </member>
    <member kind="function">
      <type>To</type>
      <name>bitCast</name>
      <anchorfile>namespaceCorrade_1_1Utility.html</anchorfile>
      <anchor>a3831cece822dfde455336664676f6867</anchor>
      <arglist>(const From &amp;from)</arglist>
    </member>
  </compound>
  <compound kind="namespace">
    <name>Corrade::Utility::Directory</name>
    <filename>namespaceCorrade_1_1Utility_1_1Directory.html</filename>
    <member kind="typedef">
      <type>Containers::EnumSet&lt; Flag &gt;</type>
      <name>Flags</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Directory.html</anchorfile>
      <anchor>a9df63241a8c57cdd64316a82bc9713b5</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumeration">
      <type></type>
      <name>Flag</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Directory.html</anchorfile>
      <anchor>a60fc75369f2d2d700d1cc758261cef35</anchor>
      <arglist></arglist>
      <enumvalue file="namespaceCorrade_1_1Utility_1_1Directory.html" anchor="a60fc75369f2d2d700d1cc758261cef35ad6186b962ccb090e812467363c008e0a">SkipDotAndDotDot</enumvalue>
      <enumvalue file="namespaceCorrade_1_1Utility_1_1Directory.html" anchor="a60fc75369f2d2d700d1cc758261cef35ab5fc8c3926425785013c492ab8ecc22f">SkipFiles</enumvalue>
      <enumvalue file="namespaceCorrade_1_1Utility_1_1Directory.html" anchor="a60fc75369f2d2d700d1cc758261cef35a9011abdfaf22b80f4ae98a3f59fed46b">SkipDirectories</enumvalue>
      <enumvalue file="namespaceCorrade_1_1Utility_1_1Directory.html" anchor="a60fc75369f2d2d700d1cc758261cef35a2e58e2c745e14cc46a112c48075b863e">SkipSpecial</enumvalue>
      <enumvalue file="namespaceCorrade_1_1Utility_1_1Directory.html" anchor="a60fc75369f2d2d700d1cc758261cef35a6e1549d1367460b4ab06f5575f2d427c">SortAscending</enumvalue>
      <enumvalue file="namespaceCorrade_1_1Utility_1_1Directory.html" anchor="a60fc75369f2d2d700d1cc758261cef35a12995c419287e0ef1f6f2d6ce1dfc12a">SortDescending</enumvalue>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>fromNativeSeparators</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Directory.html</anchorfile>
      <anchor>a454691f5180e746e5f37d50e5aab9f13</anchor>
      <arglist>(std::string path)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>toNativeSeparators</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Directory.html</anchorfile>
      <anchor>ae912c293390240c7c55cf36a3dd64fc3</anchor>
      <arglist>(std::string path)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>path</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Directory.html</anchorfile>
      <anchor>a222baead7128ec0cadc433e3a8c0059f</anchor>
      <arglist>(const std::string &amp;filename)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>filename</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Directory.html</anchorfile>
      <anchor>a07df998aa551057785f2db6c51e78e63</anchor>
      <arglist>(const std::string &amp;filename)</arglist>
    </member>
    <member kind="function">
      <type>std::pair&lt; std::string, std::string &gt;</type>
      <name>splitExtension</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Directory.html</anchorfile>
      <anchor>a5fd58770313dc7c790213e18dffa3255</anchor>
      <arglist>(const std::string &amp;path)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>join</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Directory.html</anchorfile>
      <anchor>aa1c57ae6d2c15c7507ad1ae70810d4de</anchor>
      <arglist>(const std::string &amp;path, const std::string &amp;filename)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>join</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Directory.html</anchorfile>
      <anchor>a7caa7ac094864b221e993f045827dc29</anchor>
      <arglist>(std::initializer_list&lt; std::string &gt; paths)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>mkpath</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Directory.html</anchorfile>
      <anchor>ad80859f373fbf1ed39b11eb27649c34b</anchor>
      <arglist>(const std::string &amp;path)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>rm</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Directory.html</anchorfile>
      <anchor>ac78c147e3378045ca84362dc1617a5d2</anchor>
      <arglist>(const std::string &amp;path)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>move</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Directory.html</anchorfile>
      <anchor>aa3505c07378a35988f43dca3496f9631</anchor>
      <arglist>(const std::string &amp;oldPath, const std::string &amp;newPath)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>exists</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Directory.html</anchorfile>
      <anchor>a0299a9e5b18b4187615c2fd8e256d586</anchor>
      <arglist>(const std::string &amp;filename)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>isSandboxed</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Directory.html</anchorfile>
      <anchor>a3cc7a87483e3b192253cbe72d1cb1f98</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>current</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Directory.html</anchorfile>
      <anchor>ae4f90da841b0501c64f42f062b68c1e6</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>executableLocation</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Directory.html</anchorfile>
      <anchor>a4e3dbc86c7b8d0c3c39d9802a9947b35</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>home</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Directory.html</anchorfile>
      <anchor>a8dcea7e885d165d77f8ef68814788bd9</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>configurationDir</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Directory.html</anchorfile>
      <anchor>a73ddfdc8249b1bc779e8bb58cd9c319a</anchor>
      <arglist>(const std::string &amp;name)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>tmp</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Directory.html</anchorfile>
      <anchor>a7b3ca5990decccd118261fe86824af70</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; std::string &gt;</type>
      <name>list</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Directory.html</anchorfile>
      <anchor>a27708f99ad155cdf20dbfb7926478582</anchor>
      <arglist>(const std::string &amp;path, Flags flags=Flags())</arglist>
    </member>
    <member kind="function">
      <type>Containers::Array&lt; char &gt;</type>
      <name>read</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Directory.html</anchorfile>
      <anchor>a4a9da22f34b0eb0f409b8183032b46db</anchor>
      <arglist>(const std::string &amp;filename)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>readString</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Directory.html</anchorfile>
      <anchor>afdc9b2319f9470117c4fb6b2ecf63187</anchor>
      <arglist>(const std::string &amp;filename)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>write</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Directory.html</anchorfile>
      <anchor>af3e40dd0b19be0b79e8d2bad631f9009</anchor>
      <arglist>(const std::string &amp;filename, Containers::ArrayView&lt; const void &gt; data)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>writeString</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Directory.html</anchorfile>
      <anchor>a2e31a8897f1f4d78ec7179d584ab0100</anchor>
      <arglist>(const std::string &amp;filename, const std::string &amp;data)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>append</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Directory.html</anchorfile>
      <anchor>aab260374e6387877a6648dfa8696c1b3</anchor>
      <arglist>(const std::string &amp;filename, Containers::ArrayView&lt; const void &gt; data)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>appendString</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Directory.html</anchorfile>
      <anchor>a5dbaa371ae23637c6952fa1360de0f0e</anchor>
      <arglist>(const std::string &amp;filename, const std::string &amp;data)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>copy</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Directory.html</anchorfile>
      <anchor>a665cc812333bddf0c9640376291a8bcb</anchor>
      <arglist>(const std::string &amp;from, const std::string &amp;to)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>dllLocation</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Directory.html</anchorfile>
      <anchor>a0539be6affb5bd8aa9dee55a47214521</anchor>
      <arglist>(const char *name)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>fileExists</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Directory.html</anchorfile>
      <anchor>a1a83e17a2230dd6c04d33dac2a6e745f</anchor>
      <arglist>(const std::string &amp;filename)</arglist>
    </member>
    <member kind="function">
      <type>Containers::Array&lt; char, MapDeleter &gt;</type>
      <name>map</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Directory.html</anchorfile>
      <anchor>a769141becd7eed77b15c0d0d1a79cc04</anchor>
      <arglist>(const std::string &amp;filename, std::size_t size)</arglist>
    </member>
    <member kind="function">
      <type>Containers::Array&lt; const char, MapDeleter &gt;</type>
      <name>mapRead</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Directory.html</anchorfile>
      <anchor>a84c45342a8cec2111853f51873d82c9d</anchor>
      <arglist>(const std::string &amp;filename)</arglist>
    </member>
  </compound>
  <compound kind="namespace">
    <name>Corrade::Utility::String</name>
    <filename>namespaceCorrade_1_1Utility_1_1String.html</filename>
    <member kind="function">
      <type>std::string</type>
      <name>ltrim</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>a5dcd2f1e07dfeb6b3f523b8fd7fcc0db</anchor>
      <arglist>(std::string string)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>rtrim</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>a13384c2eb91136a2282cabcf6d40e7e7</anchor>
      <arglist>(std::string string)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>trim</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>ab1422c3f09858ed4959d7ddd022c913a</anchor>
      <arglist>(std::string string)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>ltrimInPlace</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>aa559956eafb291ec822be7a090303d93</anchor>
      <arglist>(std::string &amp;string)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rtrimInPlace</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>ae415f2a9274c946aa64ce76b2978f512</anchor>
      <arglist>(std::string &amp;string)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>trimInPlace</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>a9ad609584914e7de5a25fc01d807731b</anchor>
      <arglist>(std::string &amp;string)</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; std::string &gt;</type>
      <name>splitWithoutEmptyParts</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>ab0ad1369cf991d4a24ad9eb546da5994</anchor>
      <arglist>(const std::string &amp;string)</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; std::string &gt;</type>
      <name>split</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>ac33538e87b593ea3025b97cb9c4f9c7d</anchor>
      <arglist>(const std::string &amp;string, char delimiter)</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; std::string &gt;</type>
      <name>splitWithoutEmptyParts</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>a223a1c3ccc10a0a1c0d3cea7e1bb8770</anchor>
      <arglist>(const std::string &amp;string, char delimiter)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>join</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>a9af3018407a9cac247c09da9ac8e577d</anchor>
      <arglist>(const std::vector&lt; std::string &gt; &amp;strings, char delimiter)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>joinWithoutEmptyParts</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>ac2adb8eafce8ddb6f7c163bd112e5c57</anchor>
      <arglist>(const std::vector&lt; std::string &gt; &amp;strings, char delimiter)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>lowercase</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>a09fde2ac2c5c7a28e0c694d22ca89ee6</anchor>
      <arglist>(std::string string)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>uppercase</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>a4593d63bc7b1f340584b21d6a169afe9</anchor>
      <arglist>(std::string string)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>fromArray</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>a3e5a23b4c3afe46b19c0ea93a098e634</anchor>
      <arglist>(const char *string)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>fromArray</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>acb24339bd65dda380b696d363f4cc799</anchor>
      <arglist>(const char *string, std::size_t length)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>ltrim</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>afc39c7e3c0d3da5bb4cfdf4bada808ab</anchor>
      <arglist>(std::string string, const std::string &amp;characters)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>ltrim</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>a9b120b72f24ac6c1266d60b112a131f6</anchor>
      <arglist>(std::string string, const char(&amp;characters)[size])</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>rtrim</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>aa7519775e721a492a57c3752b6e0a5ae</anchor>
      <arglist>(std::string string, const std::string &amp;characters)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>rtrim</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>a787500cae52742d206c30a00d315bd88</anchor>
      <arglist>(std::string string, const char(&amp;characters)[size])</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>trim</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>a010626127bd50cc902768b3fa50e8e8c</anchor>
      <arglist>(std::string string, const std::string &amp;characters)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>trim</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>a3e6ed45cb59343ff1c968cc7c5f74864</anchor>
      <arglist>(std::string string, const char(&amp;characters)[size])</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>ltrimInPlace</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>a9ec8fbf7b2a097997d9ae0b7fd8008b6</anchor>
      <arglist>(std::string &amp;string, const std::string &amp;characters)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>ltrimInPlace</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>aeb380f7b1a78cde7b825a2dd70d13035</anchor>
      <arglist>(std::string &amp;string, const char(&amp;characters)[size])</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rtrimInPlace</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>a7fe6b0a142484d4768e6cbb915229a7e</anchor>
      <arglist>(std::string &amp;string, const std::string &amp;characters)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rtrimInPlace</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>afe0d71afe06d476c99ca801c4a60a13c</anchor>
      <arglist>(std::string &amp;string, const char(&amp;characters)[size])</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>trimInPlace</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>a60ffdcb3d21f7d4fcf1068be07c37908</anchor>
      <arglist>(std::string &amp;string, const std::string &amp;characters)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>trimInPlace</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>a03e7318d0f0755c771da20b917c294e4</anchor>
      <arglist>(std::string &amp;string, const char(&amp;characters)[size])</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; std::string &gt;</type>
      <name>splitWithoutEmptyParts</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>ac0ff6b03a281fecebe8540c3aa86cd47</anchor>
      <arglist>(const std::string &amp;string, const std::string &amp;delimiters)</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; std::string &gt;</type>
      <name>splitWithoutEmptyParts</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>a1360ec5f0e3aa9a34af046480356292f</anchor>
      <arglist>(const std::string &amp;string, const char(&amp;delimiters)[size])</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>beginsWith</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>add091001179ac1455795365a3b667073</anchor>
      <arglist>(const std::string &amp;string, const std::string &amp;prefix)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>beginsWith</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>aca63ac7667c9b71ce5e3240daf32316e</anchor>
      <arglist>(const std::string &amp;string, const char(&amp;prefix)[size])</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>beginsWith</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>adcf054be72915ea30008949ba8d7e45a</anchor>
      <arglist>(const std::string &amp;string, char prefix)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>viewBeginsWith</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>aac2a0377274984a88d8c5ba1267bfaa0</anchor>
      <arglist>(Containers::ArrayView&lt; const char &gt; string, const char(&amp;prefix)[size])</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>viewBeginsWith</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>aba910ca698919200907a1b82b446e0cb</anchor>
      <arglist>(Containers::ArrayView&lt; const char &gt; string, char prefix)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>endsWith</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>a810b5cb07e3a5e5063145ef47567ab7f</anchor>
      <arglist>(const std::string &amp;string, const std::string &amp;suffix)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>endsWith</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>ace94f739418cde7292a1e4ffdb75598e</anchor>
      <arglist>(const std::string &amp;string, const char(&amp;suffix)[size])</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>endsWith</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>a5a89d25c3d54527384d85f3b9b22010f</anchor>
      <arglist>(const std::string &amp;string, char suffix)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>viewEndsWith</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>a036c4916e1cd32b0d31978899c205a9c</anchor>
      <arglist>(Containers::ArrayView&lt; const char &gt; string, const char(&amp;suffix)[size])</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>viewEndsWith</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>a86b3902899551c11f8a1db1b9b0becdf</anchor>
      <arglist>(Containers::ArrayView&lt; const char &gt; string, char suffix)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>stripPrefix</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>a6d1d8723eecd80d2cb0b476a92730782</anchor>
      <arglist>(std::string string, const std::string &amp;prefix)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>stripPrefix</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>a220d3903694aa0a90ec23540054c73cd</anchor>
      <arglist>(std::string string, const char(&amp;prefix)[size])</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>stripPrefix</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>a6dbe4d041b412de515dfaf992f6778b0</anchor>
      <arglist>(std::string string, char prefix)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>stripSuffix</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>a4913f68a09a3ad6dec44e8220d8d4c72</anchor>
      <arglist>(std::string string, const std::string &amp;suffix)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>stripSuffix</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>ab70c0084c70b8dfc512283edb553c5fa</anchor>
      <arglist>(std::string string, const char(&amp;suffix)[size])</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>stripSuffix</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>a376d68bfe85b7c6a6d4a4d87209fcd2b</anchor>
      <arglist>(std::string string, char suffix)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>replaceFirst</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>ae74814f4438913aa1300538529069d32</anchor>
      <arglist>(std::string string, const std::string &amp;search, const std::string &amp;replace)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>replaceFirst</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>a4efbec885c01308ee148007ad21d42ad</anchor>
      <arglist>(std::string string, const char(&amp;search)[searchSize], const char(&amp;replace)[replaceSize])</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>replaceFirst</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>ac7fb659c24110adebb2955a75d290442</anchor>
      <arglist>(std::string string, const char(&amp;search)[searchSize], const std::string &amp;replace)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>replaceFirst</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>a4d54d0ac8fb49d705ae692bc88fa2c85</anchor>
      <arglist>(std::string string, const std::string &amp;search, const char(&amp;replace)[replaceSize])</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>replaceAll</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>aef5cb78b1c8e6336abce1c5b94b6fbf0</anchor>
      <arglist>(std::string string, const std::string &amp;search, const std::string &amp;replace)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>replaceAll</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>a6dd7b108947809be4906cab43e050f0b</anchor>
      <arglist>(std::string string, const char(&amp;search)[searchSize], const char(&amp;replace)[replaceSize])</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>replaceAll</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>a3c0ad6e25f8092de0ac2737ff9c62ee7</anchor>
      <arglist>(std::string string, const char(&amp;search)[searchSize], const std::string &amp;replace)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>replaceAll</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1String.html</anchorfile>
      <anchor>a47b378ccf712482676488a771ec07f46</anchor>
      <arglist>(std::string string, const std::string &amp;search, const char(&amp;replace)[replaceSize])</arglist>
    </member>
  </compound>
  <compound kind="namespace">
    <name>Corrade::Utility::System</name>
    <filename>namespaceCorrade_1_1Utility_1_1System.html</filename>
    <member kind="function">
      <type>void</type>
      <name>sleep</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1System.html</anchorfile>
      <anchor>a7f93179c285d2667b97d9b38476d4cda</anchor>
      <arglist>(std::size_t ms)</arglist>
    </member>
  </compound>
  <compound kind="namespace">
    <name>Corrade::Utility::Unicode</name>
    <filename>namespaceCorrade_1_1Utility_1_1Unicode.html</filename>
    <member kind="function">
      <type>std::pair&lt; char32_t, std::size_t &gt;</type>
      <name>nextChar</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Unicode.html</anchorfile>
      <anchor>a23fbffa5a6184ab6c29030e152182c29</anchor>
      <arglist>(Containers::ArrayView&lt; const char &gt; text, std::size_t cursor)</arglist>
    </member>
    <member kind="function">
      <type>std::pair&lt; char32_t, std::size_t &gt;</type>
      <name>nextChar</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Unicode.html</anchorfile>
      <anchor>a7689b3d817c5244f7300a8503441865a</anchor>
      <arglist>(const std::string &amp;text, const std::size_t cursor)</arglist>
    </member>
    <member kind="function">
      <type>std::pair&lt; char32_t, std::size_t &gt;</type>
      <name>prevChar</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Unicode.html</anchorfile>
      <anchor>ab5eed1f9f89a2e19eaef0c0f64fb9900</anchor>
      <arglist>(Containers::ArrayView&lt; const char &gt; text, std::size_t cursor)</arglist>
    </member>
    <member kind="function">
      <type>std::pair&lt; char32_t, std::size_t &gt;</type>
      <name>prevChar</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Unicode.html</anchorfile>
      <anchor>a73e6d76d0393312eece30aa83a5fdd73</anchor>
      <arglist>(const std::string &amp;text, const std::size_t cursor)</arglist>
    </member>
    <member kind="function">
      <type>std::size_t</type>
      <name>utf8</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Unicode.html</anchorfile>
      <anchor>a0b1547a3b650117161826a75943e5946</anchor>
      <arglist>(char32_t character, Containers::StaticArrayView&lt; 4, char &gt; result)</arglist>
    </member>
    <member kind="function">
      <type>std::u32string</type>
      <name>utf32</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Unicode.html</anchorfile>
      <anchor>a738435f73e123822a70f692d9d9cfec8</anchor>
      <arglist>(const std::string &amp;text)</arglist>
    </member>
    <member kind="function">
      <type>std::pair&lt; char32_t, std::size_t &gt;</type>
      <name>nextChar</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Unicode.html</anchorfile>
      <anchor>a634c36aab0be25e8417905c1a1b492c4</anchor>
      <arglist>(const char(&amp;text)[size], const std::size_t cursor)</arglist>
    </member>
    <member kind="function">
      <type>std::pair&lt; char32_t, std::size_t &gt;</type>
      <name>prevChar</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Unicode.html</anchorfile>
      <anchor>ab7b8f89831f52d67d311c5886b847ce6</anchor>
      <arglist>(const char(&amp;text)[size], const std::size_t cursor)</arglist>
    </member>
    <member kind="function">
      <type>std::wstring</type>
      <name>widen</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Unicode.html</anchorfile>
      <anchor>af92cf003d4c9bf36b0a53777ab5a0445</anchor>
      <arglist>(const std::string &amp;text)</arglist>
    </member>
    <member kind="function">
      <type>std::wstring</type>
      <name>widen</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Unicode.html</anchorfile>
      <anchor>aa4f142c6268dbd30d3b816e36e9ab37b</anchor>
      <arglist>(Containers::ArrayView&lt; const char &gt; text)</arglist>
    </member>
    <member kind="function">
      <type>std::wstring</type>
      <name>widen</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Unicode.html</anchorfile>
      <anchor>a025f6762f095b8180ce0569573b82380</anchor>
      <arglist>(const char *text)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>narrow</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Unicode.html</anchorfile>
      <anchor>ad81ae601a86426189543698218fb276d</anchor>
      <arglist>(const std::wstring &amp;text)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>narrow</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Unicode.html</anchorfile>
      <anchor>a68b980fd21405108c7eb80d1b07408e6</anchor>
      <arglist>(Containers::ArrayView&lt; const wchar_t &gt; text)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>narrow</name>
      <anchorfile>namespaceCorrade_1_1Utility_1_1Unicode.html</anchorfile>
      <anchor>a7dfc7f84250f3a9a16f4a8fb4bc7a97b</anchor>
      <arglist>(const wchar_t *text)</arglist>
    </member>
  </compound>
  <compound kind="namespace">
    <name>std</name>
    <filename>namespacestd.html</filename>
  </compound>
  <compound kind="page">
    <name>building-corrade</name>
    <title>Downloading and building Corrade</title>
    <filename>building-corrade</filename>
    <docanchor file="building-corrade" title="Single-header libraries">building-corrade-singles</docanchor>
    <docanchor file="building-corrade" title="Prepared packages">building-corrade-packages</docanchor>
    <docanchor file="building-corrade" title="Conan package">building-corrade-packages-conan</docanchor>
    <docanchor file="building-corrade" title="Hunter package">building-corrade-packages-hunter</docanchor>
    <docanchor file="building-corrade" title="Vcpkg packages on Windows">building-corrade-packages-vcpkg</docanchor>
    <docanchor file="building-corrade" title="ArchLinux packages">building-corrade-packages-arch</docanchor>
    <docanchor file="building-corrade" title="MSYS2 packages">building-corrade-packages-msys</docanchor>
    <docanchor file="building-corrade" title="Packages for Debian, Ubuntu and derivatives">building-corrade-packages-deb</docanchor>
    <docanchor file="building-corrade" title="Gentoo Linux ebuilds">building-corrade-packages-gentoo</docanchor>
    <docanchor file="building-corrade" title="Packages for Fedora, openSUSE and other RPM-based Linux distributions">building-corrade-packages-rpm</docanchor>
    <docanchor file="building-corrade" title="Homebrew formulas for macOS">building-corrade-packages-brew</docanchor>
    <docanchor file="building-corrade" title="Manual build">building-corrade-manual</docanchor>
    <docanchor file="building-corrade" title="Downloading the sources">building-corrade-download</docanchor>
    <docanchor file="building-corrade" title="CMake primer">building-corrade-cmake</docanchor>
    <docanchor file="building-corrade" title="Via command line (on Linux/Unix)">building-corrade-linux</docanchor>
    <docanchor file="building-corrade" title="Building on Windows">building-corrade-windows</docanchor>
    <docanchor file="building-corrade" title="Using Visual Studio">building-corrade-windows-msvc</docanchor>
    <docanchor file="building-corrade" title="Using QtCreator">building-corrade-windows-qtcreator</docanchor>
    <docanchor file="building-corrade" title="Enabling or disabling features">building-corrade-features</docanchor>
    <docanchor file="building-corrade" title="Building and running tests">building-corrade-tests</docanchor>
    <docanchor file="building-corrade" title="Building documentation">building-corrade-doc</docanchor>
    <docanchor file="building-corrade" title="Building examples">building-corrade-examples</docanchor>
    <docanchor file="building-corrade" title="Cross-compiling">building-corrade-cross</docanchor>
    <docanchor file="building-corrade" title="Cross-compiling for Windows RT">building-corrade-cross-winrt</docanchor>
    <docanchor file="building-corrade" title="Cross-compiling for Windows using MinGW-w64">building-corrade-cross-win</docanchor>
    <docanchor file="building-corrade" title="Cross-compiling for Emscripten">building-corrade-cross-emscripten</docanchor>
    <docanchor file="building-corrade" title="Cross-compiling for iOS">building-corrade-cross-ios</docanchor>
    <docanchor file="building-corrade" title="Cross-compiling for Android">building-corrade-cross-android</docanchor>
    <docanchor file="building-corrade" title="Continuous Integration">building-corrade-ci</docanchor>
    <docanchor file="building-corrade" title="Travis">building-corrade-ci-travis</docanchor>
    <docanchor file="building-corrade" title="AppVeyor">building-corrade-ci-appveyor</docanchor>
    <docanchor file="building-corrade" title="Codecov.io">building-corrade-ci-coverage</docanchor>
  </compound>
  <compound kind="page">
    <name>corrade-cmake</name>
    <title>Using Corrade with CMake</title>
    <filename>corrade-cmake</filename>
    <docanchor file="corrade-cmake" title="Other CMake modules">corrade-cmake-modules</docanchor>
    <docanchor file="corrade-cmake" title="Macros and functions">corrade-cmake-functions</docanchor>
    <docanchor file="corrade-cmake" title="Add unit test using Corrade&apos;s TestSuite">corrade-cmake-add-test</docanchor>
    <docanchor file="corrade-cmake" title="Compile data resources into application binary">corrade-cmake-add-resource</docanchor>
    <docanchor file="corrade-cmake" title="Add dynamic plugin">corrade-cmake-add-plugin</docanchor>
    <docanchor file="corrade-cmake" title="Add static plugin">corrade-cmake-add-static-plugin</docanchor>
    <docanchor file="corrade-cmake" title="Find corresponding DLLs for library files">corrade-cmake-find-dlls-for-libs</docanchor>
  </compound>
  <compound kind="page">
    <name>main</name>
    <title>Corrade::Main library</title>
    <filename>main</filename>
    <docanchor file="main" title="Standard main() with UTF-8 command-line arguments">main-utf8-arguments</docanchor>
    <docanchor file="main" title="WIN32 apps without console window lurking in the background">main-winmain</docanchor>
    <docanchor file="main" title="ANSI colors in console output">main-ansi-colors</docanchor>
    <docanchor file="main" title="UTF-8 console output encoding">main-utf8-output</docanchor>
    <docanchor file="main" title="Usage without CMake">main-non-cmake</docanchor>
  </compound>
  <compound kind="page">
    <name>corrade-singles</name>
    <title>Single-header libraries</title>
    <filename>corrade-singles</filename>
    <docanchor file="corrade-singles" title="Behavior">corrade-singles-behavior</docanchor>
    <docanchor file="corrade-singles" title="Customization points">corrade-singles-customization</docanchor>
  </compound>
  <compound kind="page">
    <name>corrade-example-index</name>
    <title>Examples</title>
    <filename>corrade-example-index</filename>
  </compound>
  <compound kind="page">
    <name>corrade-rc</name>
    <title>Resource compiler</title>
    <filename>corrade-rc</filename>
    <docanchor file="corrade-rc" title="Usage">corrade-rc-usage</docanchor>
  </compound>
  <compound kind="page">
    <name>corrade-changelog</name>
    <title>Changelog</title>
    <filename>corrade-changelog</filename>
    <docanchor file="corrade-changelog" title="Changes since 2019.01">corrade-changelog-latest</docanchor>
    <docanchor file="corrade-changelog" title="New features">corrade-changelog-latest-new</docanchor>
    <docanchor file="corrade-changelog" title="Containers library">corrade-changelog-latest-new-containers</docanchor>
    <docanchor file="corrade-changelog" title="TestSuite library">corrade-changelog-latest-new-testsuite</docanchor>
    <docanchor file="corrade-changelog" title="Utility library">corrade-changelog-latest-new-utility</docanchor>
    <docanchor file="corrade-changelog" title="Changes and improvements">corrade-changelog-latest-changes</docanchor>
    <docanchor file="corrade-changelog" title="Containers library">corrade-changelog-latest-changes-containers</docanchor>
    <docanchor file="corrade-changelog" title="Interconnect library">corrade-changelog-latest-changes-Interconnect</docanchor>
    <docanchor file="corrade-changelog" title="PluginManager library">corrade-changelog-latest-changes-pluginmanager</docanchor>
    <docanchor file="corrade-changelog" title="TestSuite library">corrade-changelog-latest-changes-testsuite</docanchor>
    <docanchor file="corrade-changelog" title="Utility library">corrade-changelog-latest-changes-utility</docanchor>
    <docanchor file="corrade-changelog" title="Build system">corrade-changelog-latest-buildsystem</docanchor>
    <docanchor file="corrade-changelog" title="Bug fixes">corrade-changelog-latest-bugfixes</docanchor>
    <docanchor file="corrade-changelog" title="Deprecated APIs">corrade-changelog-latest-deprecated</docanchor>
    <docanchor file="corrade-changelog" title="Potential compatibility breakages, removed APIs">corrade-changelog-latest-compatibility</docanchor>
    <docanchor file="corrade-changelog" title="Documentation">corrade-changelog-latest-documentation</docanchor>
    <docanchor file="corrade-changelog" title="2019.01">corrade-changelog-2019-01</docanchor>
    <docanchor file="corrade-changelog" title="Dependency changes">corrade-changelog-2019-01-dependencies</docanchor>
    <docanchor file="corrade-changelog" title="New features">corrade-changelog-2019-01-new</docanchor>
    <docanchor file="corrade-changelog" title="Containers library">corrade-changelog-2019-01-new-containers</docanchor>
    <docanchor file="corrade-changelog" title="Utility library">corrade-changelog-2019-01-new-utility</docanchor>
    <docanchor file="corrade-changelog" title="Changes and improvements">corrade-changelog-2019-01-changes</docanchor>
    <docanchor file="corrade-changelog" title="Containers library">corrade-changelog-2019-01-changes-containers</docanchor>
    <docanchor file="corrade-changelog" title="PluginManager library">corrade-changelog-2019-01-changes-pluginmanager</docanchor>
    <docanchor file="corrade-changelog" title="Utility library">corrade-changelog-2019-01-changes-utility</docanchor>
    <docanchor file="corrade-changelog" title="Build system">corrade-changelog-2019-01-buildsystem</docanchor>
    <docanchor file="corrade-changelog" title="Bug fixes">corrade-changelog-2019-01-bugfixes</docanchor>
    <docanchor file="corrade-changelog" title="Deprecated APIs">corrade-changelog-2019-01-deprecated</docanchor>
    <docanchor file="corrade-changelog" title="Potential compatibility breakages, removed APIs">corrade-changelog-2019-01-compatibility</docanchor>
    <docanchor file="corrade-changelog" title="Documentation">corrade-changelog-2019-01-documentation</docanchor>
    <docanchor file="corrade-changelog" title="2018.10">corrade-changelog-2018-10</docanchor>
    <docanchor file="corrade-changelog" title="New features">corrade-changelog-2018-10-new</docanchor>
    <docanchor file="corrade-changelog" title="Containers library">corrade-changelog-2018-10-new-containers</docanchor>
    <docanchor file="corrade-changelog" title="PluginManager library">corrade-changelog-2018-10-new-pluginmanager</docanchor>
    <docanchor file="corrade-changelog" title="Utility library">corrade-changelog-2018-10-new-utility</docanchor>
    <docanchor file="corrade-changelog" title="Changes and improvements">corrade-changelog-2018-10-changes</docanchor>
    <docanchor file="corrade-changelog" title="Containers library">corrade-changelog-2018-10-changes-containers</docanchor>
    <docanchor file="corrade-changelog" title="PluginManager library">corrade-changelog-2018-10-changes-pluginmanager</docanchor>
    <docanchor file="corrade-changelog" title="TestSuite library">corrade-changelog-2018-10-changes-testsuite</docanchor>
    <docanchor file="corrade-changelog" title="Utility library">corrade-changelog-2018-10-changes-utility</docanchor>
    <docanchor file="corrade-changelog" title="Build system">corrade-changelog-2018-10-buildsystem</docanchor>
    <docanchor file="corrade-changelog" title="Bug fixes">corrade-changelog-2018-10-bugfixes</docanchor>
    <docanchor file="corrade-changelog" title="Documentation">corrade-changelog-2018-10-docs</docanchor>
    <docanchor file="corrade-changelog" title="2018.04">corrade-changelog-2018-04</docanchor>
    <docanchor file="corrade-changelog" title="Dependency changes">corrade-changelog-2018-04-dependencies</docanchor>
    <docanchor file="corrade-changelog" title="New features">corrade-changelog-2018-04-new</docanchor>
    <docanchor file="corrade-changelog" title="PluginManager library">corrade-changelog-2018-04-new-pluginmanager</docanchor>
    <docanchor file="corrade-changelog" title="TestSuite library">corrade-changelog-2018-04-new-testsuite</docanchor>
    <docanchor file="corrade-changelog" title="Utility library">corrade-changelog-2018-04-new-utility</docanchor>
    <docanchor file="corrade-changelog" title="Build system">corrade-changelog-2018-04-buildsystem</docanchor>
    <docanchor file="corrade-changelog" title="Bug fixes">corrade-changelog-2018-04-bugfixes</docanchor>
    <docanchor file="corrade-changelog" title="Deprecated APIs">corrade-changelog-2018-04-deprecated</docanchor>
    <docanchor file="corrade-changelog" title="Potential compatibility breakages, removed APIs">corrade-changelog-2018-04-compatibility</docanchor>
    <docanchor file="corrade-changelog" title="Documentation">corrade-changelog-2018-04-docs</docanchor>
    <docanchor file="corrade-changelog" title="2018.02">corrade-changelog-2018-02</docanchor>
    <docanchor file="corrade-changelog" title="Dependency changes">corrade-changelog-2018-02-dependencies</docanchor>
    <docanchor file="corrade-changelog" title="New features">corrade-changelog-2018-02-new</docanchor>
    <docanchor file="corrade-changelog" title="Containers library">corrade-changelog-2018-02-new-containers</docanchor>
    <docanchor file="corrade-changelog" title="PluginManager library">corrade-changelog-2018-02-new-pluginmanager</docanchor>
    <docanchor file="corrade-changelog" title="TestSuite library">corrade-changelog-2018-02-new-testsuite</docanchor>
    <docanchor file="corrade-changelog" title="Utility library">corrade-changelog-2018-02-new-utility</docanchor>
    <docanchor file="corrade-changelog" title="Changes and improvements">corrade-changelog-2018-02-changes</docanchor>
    <docanchor file="corrade-changelog" title="Containers library">corrade-changelog-2018-02-changes-containers</docanchor>
    <docanchor file="corrade-changelog" title="PluginManager library">corrade-changelog-2018-02-changes-pluginmanager</docanchor>
    <docanchor file="corrade-changelog" title="TestSuite library">corrade-changelog-2018-02-changes-testsuite</docanchor>
    <docanchor file="corrade-changelog" title="Utility library">corrade-changelog-2018-02-changes-utility</docanchor>
    <docanchor file="corrade-changelog" title="Build system">corrade-changelog-2018-02-buildsystem</docanchor>
    <docanchor file="corrade-changelog" title="Bug fixes">corrade-changelog-2018-02-bugfixes</docanchor>
    <docanchor file="corrade-changelog" title="Deprecated APIs">corrade-changelog-2018-02-deprecated</docanchor>
    <docanchor file="corrade-changelog" title="Potential compatibility breakages, removed APIs">corrade-changelog-2018-02-compatibility</docanchor>
    <docanchor file="corrade-changelog" title="Documentation">corrade-changelog-2018-02-documentation</docanchor>
  </compound>
  <compound kind="page">
    <name>corrade-changelog-old</name>
    <title>Archived changelogs</title>
    <filename>corrade-changelog-old</filename>
    <docanchor file="corrade-changelog-old" title="2015.05">corrade-changelog-2015-05</docanchor>
    <docanchor file="corrade-changelog-old" title="Dependency changes">corrade-changelog-2015-05-dependencies</docanchor>
    <docanchor file="corrade-changelog-old" title="New features">corrade-changelog-2015-05-new</docanchor>
    <docanchor file="corrade-changelog-old" title="Changes">corrade-changelog-2015-05-changes</docanchor>
    <docanchor file="corrade-changelog-old" title="Build system">corrade-changelog-2015-05-buildsystem</docanchor>
    <docanchor file="corrade-changelog-old" title="Bug fixes">corrade-changelog-2015-05-bugfixes</docanchor>
    <docanchor file="corrade-changelog-old" title="Deprecated APIs">corrade-changelog-2015-05-deprecated</docanchor>
    <docanchor file="corrade-changelog-old" title="Potential compatibility breakages, removed APIs">corrade-changelog-2015-05-compatibility</docanchor>
    <docanchor file="corrade-changelog-old" title="2014.06">corrade-changelog-2014-06</docanchor>
    <docanchor file="corrade-changelog-old" title="Dependency changes">corrade-changelog-2014-06-dependencies</docanchor>
    <docanchor file="corrade-changelog-old" title="New features">corrade-changelog-2014-06-new</docanchor>
    <docanchor file="corrade-changelog-old" title="Changes">corrade-changelog-2014-06-changes</docanchor>
    <docanchor file="corrade-changelog-old" title="Bug fixes">corrade-changelog-2014-06-bugfixes</docanchor>
    <docanchor file="corrade-changelog-old" title="Deprecated APIs">corrade-changelog-2014-06-deprecated</docanchor>
    <docanchor file="corrade-changelog-old" title="Potential compatibility breakages, removed APIs">corrade-changelog-2014-06-compatibility</docanchor>
    <docanchor file="corrade-changelog-old" title="2014.01">corrade-changelog-2014-01</docanchor>
    <docanchor file="corrade-changelog-old" title="Dependency changes">corrade-changelog-2014-01-dependencies</docanchor>
    <docanchor file="corrade-changelog-old" title="New features">corrade-changelog-2014-01-new</docanchor>
    <docanchor file="corrade-changelog-old" title="Changes">corrade-changelog-2014-01-changes</docanchor>
    <docanchor file="corrade-changelog-old" title="Bug fixes">corrade-changelog-2014-01-bugfixes</docanchor>
    <docanchor file="corrade-changelog-old" title="Deprecated APIs">corrade-changelog-2014-01-deprecated</docanchor>
    <docanchor file="corrade-changelog-old" title="Potential compatibility breakages, removed APIs">corrade-changelog-2014-01-compatibility</docanchor>
    <docanchor file="corrade-changelog-old" title="Internal changes">corrade-changelog-2014-01-internal</docanchor>
    <docanchor file="corrade-changelog-old" title="2013.10">corrade-changelog-2013-10</docanchor>
    <docanchor file="corrade-changelog-old" title="Dependency changes">corrade-changelog-2013-10-dependencies</docanchor>
    <docanchor file="corrade-changelog-old" title="New features">corrade-changelog-2013-10-new</docanchor>
    <docanchor file="corrade-changelog-old" title="Changes">corrade-changelog-2013-10-changes</docanchor>
    <docanchor file="corrade-changelog-old" title="Build system">corrade-changelog-2013-10-buildsystem</docanchor>
    <docanchor file="corrade-changelog-old" title="Bug fixes">corrade-changelog-2013-10-bugfixes</docanchor>
    <docanchor file="corrade-changelog-old" title="Deprecated APIs">corrade-changelog-2013-10-deprecated</docanchor>
    <docanchor file="corrade-changelog-old" title="Potential compatibility breakages, removed APIs">corrade-changelog-2013-10-compatibility</docanchor>
    <docanchor file="corrade-changelog-old" title="2013.08">corrade-changelog-2013-08</docanchor>
    <docanchor file="corrade-changelog-old" title="2012.02">corrade-changelog-2012-02</docanchor>
  </compound>
  <compound kind="page">
    <name>corrade-developers</name>
    <title>Developers guide</title>
    <filename>corrade-developers</filename>
    <docanchor file="corrade-developers" title="Checklist for adding / removing a library">corrade-developers-library</docanchor>
    <docanchor file="corrade-developers" title="Checklist for adding / removing a plugin">corrade-developers-plugin</docanchor>
    <docanchor file="corrade-developers" title="Checklist for adding / removing a plugin interface">corrade-developers-plugin-interface</docanchor>
    <docanchor file="corrade-developers" title="Checklist for adding / removing a tool">corrade-developers-tool</docanchor>
    <docanchor file="corrade-developers" title="Checklist for adding / removing an example">corrade-developers-example</docanchor>
    <docanchor file="corrade-developers" title="Checklist for adding / removing a new source / header file">corrade-developers-file</docanchor>
    <docanchor file="corrade-developers" title="Checklist for adding / removing a symbol">corrade-developers-symbol</docanchor>
    <docanchor file="corrade-developers" title="Checklist for adding a new CMake documentation page">corrade-developers-page</docanchor>
    <docanchor file="corrade-developers" title="Checklist for deprecating a feature">corrade-developers-deprecation</docanchor>
    <docanchor file="corrade-developers" title="Checklist for removing a feature">corrade-developers-removing</docanchor>
    <docanchor file="corrade-developers" title="Checklist for adding, removing or updating a dependency">corrade-developers-dependency</docanchor>
    <docanchor file="corrade-developers" title="Checklist for adding or removing a port">corrade-developers-port</docanchor>
    <docanchor file="corrade-developers" title="Checklist for updating copyright year">corrade-developers-copyright-year</docanchor>
    <docanchor file="corrade-developers" title="Checklist for uploading documentation">corrade-developers-documentation</docanchor>
    <docanchor file="corrade-developers" title="Checklist for merging a PR">corrade-developers-pr</docanchor>
    <docanchor file="corrade-developers" title="Checklist for creating/updating a single-header lib">corrade-developers-singles</docanchor>
    <docanchor file="corrade-developers" title="Checklist for making a release">corrade-developers-release</docanchor>
  </compound>
  <compound kind="page">
    <name>acme</name>
    <title>Single-header generator tool</title>
    <filename>acme</filename>
    <docanchor file="acme" title="System include placement">acme-system-includes</docanchor>
    <docanchor file="acme" title="Local include matching">acme-local-include-matching</docanchor>
    <docanchor file="acme" title="Local include processing">acme-local-include-processing</docanchor>
    <docanchor file="acme" title="Preprocessor branch processing">acme-preprocessor</docanchor>
    <docanchor file="acme" title="Copyright information">acme-copyright</docanchor>
    <docanchor file="acme" title="Code comments">acme-comments</docanchor>
    <docanchor file="acme" title="{{revision}}">acme-revision</docanchor>
    <docanchor file="acme" title="{{stats}}">acme-stats</docanchor>
    <docanchor file="acme" title="Real-world examples">acme-example</docanchor>
  </compound>
  <compound kind="page">
    <name>corrade-coding-style</name>
    <title>Coding style</title>
    <filename>corrade-coding-style</filename>
    <docanchor file="corrade-coding-style" title="Text files in general">corrade-coding-style-text</docanchor>
    <docanchor file="corrade-coding-style" title="CMake code">corrade-coding-style-cmake</docanchor>
    <docanchor file="corrade-coding-style" title="CMake modules">corrade-coding-style-cmake-modules</docanchor>
    <docanchor file="corrade-coding-style" title="C++ code">corrade-coding-style-cpp</docanchor>
    <docanchor file="corrade-coding-style" title="File and directory naming">corrade-coding-style-cpp-filesystem</docanchor>
    <docanchor file="corrade-coding-style" title="File structure">corrade-coding-style-cpp-files</docanchor>
    <docanchor file="corrade-coding-style" title="Code format">corrade-coding-style-cpp-format</docanchor>
    <docanchor file="corrade-coding-style" title="Naming">corrade-coding-style-cpp-naming</docanchor>
    <docanchor file="corrade-coding-style" title="Forward declarations and forward declaration headers">corrade-coding-style-cpp-forward-declarations</docanchor>
    <docanchor file="corrade-coding-style" title="Namespace declarations">corrade-coding-style-cpp-namespace</docanchor>
    <docanchor file="corrade-coding-style" title="Class and structure declarations">corrade-coding-style-cpp-classes</docanchor>
    <docanchor file="corrade-coding-style" title="Blocks, whitespace and indentation">corrade-coding-style-cpp-style</docanchor>
    <docanchor file="corrade-coding-style" title="Switch statements">corrade-coding-style-cpp-switches</docanchor>
    <docanchor file="corrade-coding-style" title="Class member and function keywords">corrade-coding-style-cpp-keywords</docanchor>
    <docanchor file="corrade-coding-style" title="Preprocessor macros">corrade-coding-style-cpp-macros</docanchor>
    <docanchor file="corrade-coding-style" title="Class constructors and destructors">corrade-coding-style-cpp-constructors</docanchor>
    <docanchor file="corrade-coding-style" title="Constant expressions and constants">corrade-coding-style-cpp-constexpr</docanchor>
    <docanchor file="corrade-coding-style" title="Assertions">corrade-coding-style-cpp-assert</docanchor>
    <docanchor file="corrade-coding-style" title="Enums and inheritance">corrade-coding-style-cpp-enum-inheritance</docanchor>
    <docanchor file="corrade-coding-style" title="Initialization">corrade-coding-style-cpp-initialization</docanchor>
    <docanchor file="corrade-coding-style" title="Virtual functions">corrade-coding-style-cpp-virtual</docanchor>
    <docanchor file="corrade-coding-style" title="Naked pointers">corrade-coding-style-cpp-pointers</docanchor>
    <docanchor file="corrade-coding-style" title="SFINAE and templates">corrade-coding-style-cpp-sfinae</docanchor>
    <docanchor file="corrade-coding-style" title="Discouraged C/C++ features">corrade-coding-style-cpp-discouraged</docanchor>
    <docanchor file="corrade-coding-style" title="Heavy STL headers">corrade-coding-style-cpp-heavy-headers</docanchor>
    <docanchor file="corrade-coding-style" title="using namespace keyword">corrade-coding-style-cpp-using</docanchor>
    <docanchor file="corrade-coding-style" title="C-style casts">corrade-coding-style-cpp-backward</docanchor>
    <docanchor file="corrade-coding-style" title="Unscoped and untyped enums">corrade-coding-style-cpp-enums</docanchor>
    <docanchor file="corrade-coding-style" title="static keyword">corrade-coding-style-cpp-anonymous-namespace</docanchor>
    <docanchor file="corrade-coding-style" title="Comments">corrade-coding-style-comments</docanchor>
    <docanchor file="corrade-coding-style" title="Doxygen documentation">corrade-coding-style-documentation</docanchor>
    <docanchor file="corrade-coding-style" title="Section ordering">corrade-coding-style-documentation-ordering</docanchor>
    <docanchor file="corrade-coding-style" title="Special documentation commands">corrade-coding-style-documentation-commands</docanchor>
    <docanchor file="corrade-coding-style" title="Code">corrade-coding-style-documentation-commands-code</docanchor>
    <docanchor file="corrade-coding-style" title="Documentation-related TODOs">corrade-coding-style-documentation-commands-todoc</docanchor>
    <docanchor file="corrade-coding-style" title="Debugging operators">corrade-coding-style-documentation-commands-debugoperator</docanchor>
    <docanchor file="corrade-coding-style" title="Configuration value parsers and writers">corrade-coding-style-documentation-commands-configurationvalue</docanchor>
    <docanchor file="corrade-coding-style" title="Tweakable literal parsers">corrade-coding-style-documentation-commands-tweakableliteral</docanchor>
    <docanchor file="corrade-coding-style" title="Partially supported features">corrade-coding-style-documentation-commands-partialsupport</docanchor>
    <docanchor file="corrade-coding-style" title="Third party dependency licensing info">corrade-coding-style-documentation-commands-thirdparty</docanchor>
    <docanchor file="corrade-coding-style" title="Backwards compatibility and experimental features">corrade-coding-style-documentation-commands-experimental</docanchor>
    <docanchor file="corrade-coding-style" title="Git">corrade-coding-style-git</docanchor>
    <docanchor file="corrade-coding-style" title="Commit message format">corrade-coding-style-git-commits</docanchor>
    <docanchor file="corrade-coding-style" title="Repository, branch and tag format">corrade-coding-style-git-branches</docanchor>
  </compound>
  <compound kind="page">
    <name>corrade-credits-third-party</name>
    <title>Third-party components</title>
    <filename>corrade-credits-third-party</filename>
  </compound>
  <compound kind="page">
    <name>corrade-credits-contributors</name>
    <title>Contributors</title>
    <filename>corrade-credits-contributors</filename>
  </compound>
  <compound kind="page">
    <name>corrade-configurationvalues</name>
    <title>Configuration value parsers and writers for custom types</title>
    <filename>corrade-configurationvalues</filename>
  </compound>
  <compound kind="page">
    <name>corrade-tweakableliterals</name>
    <title>Tweakable literal parsers</title>
    <filename>corrade-tweakableliterals</filename>
  </compound>
  <compound kind="page">
    <name>interconnect</name>
    <title>Signals and slots</title>
    <filename>interconnect</filename>
    <docanchor file="interconnect" title="Implementing signals">interconnect-signals</docanchor>
    <docanchor file="interconnect" title="Implementing slots">interconnect-slots</docanchor>
    <docanchor file="interconnect" title="Connecting signals to slots">interconnect-connecting</docanchor>
    <docanchor file="interconnect" title="Emitting signals">interconnect-emitting</docanchor>
    <docanchor file="interconnect" title="Compiling and running the example">interconnect-compiling</docanchor>
  </compound>
  <compound kind="page">
    <name>plugin-management</name>
    <title>Plugin management</title>
    <filename>plugin-management</filename>
    <docanchor file="plugin-management" title="Plugin interface">plugin-management-interface</docanchor>
    <docanchor file="plugin-management" title="Plugin definition">plugin-management-plugin</docanchor>
    <docanchor file="plugin-management" title="Plugin compilation">plugin-management-compilation</docanchor>
    <docanchor file="plugin-management" title="Plugin management">plugin-management-management</docanchor>
  </compound>
  <compound kind="page">
    <name>resource-management</name>
    <title>Resource management</title>
    <filename>resource-management</filename>
    <docanchor file="resource-management" title="Resource compilation">resource-management-compilation</docanchor>
    <docanchor file="resource-management" title="Resource management">resource-management-management</docanchor>
  </compound>
  <compound kind="page">
    <name>testsuite</name>
    <title>Testing and benchmarking</title>
    <filename>testsuite</filename>
    <docanchor file="testsuite" title="Tester class">testsuite-class</docanchor>
    <docanchor file="testsuite" title="Compilation and running">testsuite-compilation</docanchor>
  </compound>
  <compound kind="page">
    <name>index</name>
    <title>Corrade</title>
    <filename>index</filename>
    <docanchor file="index" title="What&apos;s new?">corrade-mainpage-whats-new</docanchor>
    <docanchor file="index" title="Getting started">corrade-mainpage-getting-started</docanchor>
    <docanchor file="index" title="Contact &amp; support">corrade-mainpage-contact</docanchor>
    <docanchor file="index" title="License">corrade-mainpage-license</docanchor>
  </compound>
</tagfile>
